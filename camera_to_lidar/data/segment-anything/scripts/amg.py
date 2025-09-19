# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

import cv2  # type: ignore

from segment_anything import SamAutomaticMaskGenerator, sam_model_registry

import argparse
import json
import os
import gc
import shutil
from typing import Any, Dict, List
import torch

torch.cuda.empty_cache()

parser = argparse.ArgumentParser(
    description=(
        "Runs automatic mask generation on an input image or directory of images, "
        "and outputs masks as either PNGs or COCO-style RLEs. Requires open-cv, "
        "as well as pycocotools if saving in RLE format."
    )
)

parser.add_argument(
    "--input",
    type=str,
    required=False,
    help="(不再使用) 脚本将自动处理 5 个 undistorted 文件夹",
)

parser.add_argument(
    "--output",
    type=str,
    required=False,
    help=(
        "Path to the directory where masks will be output. Output will be either a folder "
        "of PNGs per image or a single json with COCO-style masks."
    ),
)

parser.add_argument(
    "--model-type",
    type=str,
    required=True,
    help="The type of model to load, in ['default', 'vit_h', 'vit_l', 'vit_b']",
)

parser.add_argument(
    "--checkpoint",
    type=str,
    required=True,
    help="The path to the SAM checkpoint to use for mask generation.",
)

parser.add_argument("--device", type=str, default="cuda", help="The device to run generation on.")

parser.add_argument(
    "--convert-to-rle",
    action="store_true",
    help=(
        "Save masks as COCO RLEs in a single json instead of as a folder of PNGs. "
        "Requires pycocotools."
    ),
)

amg_settings = parser.add_argument_group("AMG Settings")

amg_settings.add_argument(
    "--points-per-side",
    type=int,
    default=None,
    help="Generate masks by sampling a grid over the image with this many points to a side.",
)

amg_settings.add_argument(
    "--points-per-batch",
    type=int,
    default=None,
    help="How many input points to process simultaneously in one batch.",
)

amg_settings.add_argument(
    "--pred-iou-thresh",
    type=float,
    default=None,
    help="Exclude masks with a predicted score from the model that is lower than this threshold.",
)

amg_settings.add_argument(
    "--stability-score-thresh",
    type=float,
    default=None,
    help="Exclude masks with a stability score lower than this threshold.",
)

amg_settings.add_argument(
    "--stability-score-offset",
    type=float,
    default=None,
    help="Larger values perturb the mask more when measuring stability score.",
)

amg_settings.add_argument(
    "--box-nms-thresh",
    type=float,
    default=None,
    help="The overlap threshold for excluding a duplicate mask.",
)

amg_settings.add_argument(
    "--crop-n-layers",
    type=int,
    default=None,
    help=(
        "If >0, mask generation is run on smaller crops of the image to generate more masks. "
        "The value sets how many different scales to crop at."
    ),
)

amg_settings.add_argument(
    "--crop-nms-thresh",
    type=float,
    default=None,
    help="The overlap threshold for excluding duplicate masks across different crops.",
)

amg_settings.add_argument(
    "--crop-overlap-ratio",
    type=int,
    default=None,
    help="Larger numbers mean image crops will overlap more.",
)

amg_settings.add_argument(
    "--crop-n-points-downscale-factor",
    type=int,
    default=None,
    help="The number of points-per-side in each layer of crop is reduced by this factor.",
)

amg_settings.add_argument(
    "--min-mask-region-area",
    type=int,
    default=None,
    help=(
        "Disconnected mask regions or holes with area smaller than this value "
        "in pixels are removed by postprocessing."
    ),
)


def write_masks_to_folder(masks: List[Dict[str, Any]], path: str) -> None:
    header = "id,area,bbox_x0,bbox_y0,bbox_w,bbox_h,point_input_x,point_input_y,predicted_iou,stability_score,crop_box_x0,crop_box_y0,crop_box_w,crop_box_h"  # noqa
    metadata = [header]
    for i, mask_data in enumerate(masks):
        mask = mask_data["segmentation"]
        filename = f"{i}.png"
        cv2.imwrite(os.path.join(path, filename), mask * 255)
        mask_metadata = [
            str(i),
            str(mask_data["area"]),
            *[str(x) for x in mask_data["bbox"]],
            *[str(x) for x in mask_data["point_coords"][0]],
            str(mask_data["predicted_iou"]),
            str(mask_data["stability_score"]),
            *[str(x) for x in mask_data["crop_box"]],
        ]
        row = ",".join(mask_metadata)
        metadata.append(row)
    metadata_path = os.path.join(path, "metadata.csv")
    with open(metadata_path, "w") as f:
        f.write("\n".join(metadata))

    return


def get_amg_kwargs(args):
    amg_kwargs = {
        "points_per_side": args.points_per_side,
        "points_per_batch": args.points_per_batch,
        "pred_iou_thresh": args.pred_iou_thresh,
        "stability_score_thresh": args.stability_score_thresh,
        "stability_score_offset": args.stability_score_offset,
        "box_nms_thresh": args.box_nms_thresh,
        "crop_n_layers": args.crop_n_layers,
        "crop_nms_thresh": args.crop_nms_thresh,
        "crop_overlap_ratio": args.crop_overlap_ratio,
        "crop_n_points_downscale_factor": args.crop_n_points_downscale_factor,
        "min_mask_region_area": args.min_mask_region_area,
    }
    amg_kwargs = {k: v for k, v in amg_kwargs.items() if v is not None}
    return amg_kwargs


def main(args: argparse.Namespace) -> None:
    print("Loading model...")
    sam = sam_model_registry[args.model_type](checkpoint=args.checkpoint)
    _ = sam.to(device=args.device)
    output_mode = "coco_rle" if args.convert_to_rle else "binary_mask"
    amg_kwargs = get_amg_kwargs(args)
    generator = SamAutomaticMaskGenerator(sam, output_mode=output_mode, **amg_kwargs)

    # 定义需要处理的 5 个 undistorted 文件夹
    undistorted_folders = [
        '../fisheye-front/undistorted',
        '../fisheye-left/undistorted', 
        '../fisheye-right/undistorted',
        '../pinhole-back/undistorted',
        '../pinhole-front/undistorted'
    ]
    output_folders = [
        '../fisheye-front/masks',
        '../fisheye-left/masks',
        '../fisheye-right/masks',
        '../pinhole-back/masks',
        '../pinhole-front/masks'
    ]
    
    # 在开始处理之前，清空所有目标的masks文件夹
    print("清空目标masks文件夹...")
    for output_folder in output_folders:
        if os.path.exists(output_folder):
            try:
                shutil.rmtree(output_folder)
                print(f"已清空文件夹: {output_folder}")
            except Exception as e:
                print(f"清空文件夹 {output_folder} 时发生错误: {e}")
        else:
            print(f"文件夹不存在，无需清空: {output_folder}")
    
    total_processed = 0
    image_extensions = ('.jpg', '.jpeg', '.png', '.bmp', '.tiff')
    
    for folder_path, output_folder in zip(undistorted_folders, output_folders):
        if not os.path.isdir(folder_path):
            print(f"警告：文件夹 '{folder_path}' 不存在，跳过...")
            continue           
        
        # 获取该文件夹中的所有图片文件
        targets = [
            f for f in os.listdir(folder_path) 
            if not os.path.isdir(os.path.join(folder_path, f)) 
            and f.lower().endswith(image_extensions)
        ]
        
        if not targets:
            print(f"警告：在文件夹 '{folder_path}' 中没有找到图片文件，跳过...")
            continue
            
        targets = [os.path.join(folder_path, f) for f in targets]
        
        print(f"\n开始处理文件夹 '{folder_path}'，包含 {len(targets)} 张图片：")
        for t in targets:
            print(f"- {os.path.basename(t)}")
            
        os.makedirs(output_folder, exist_ok=True)
        
        for t in targets:
            base = os.path.basename(t)
            base = os.path.splitext(base)[0]
            save_base = os.path.join(output_folder, base)
            
            print(f"正在处理 '{t}'...")
            image = cv2.imread(t)
            if image is None:
                print(f"无法加载图片 '{t}'，跳过...")
                continue
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            masks = generator.generate(image)
            
            if output_mode == "binary_mask":
                os.makedirs(save_base, exist_ok=True)
                write_masks_to_folder(masks, save_base)
                print(f"  -> 生成了 {len(masks)} 个掩码，保存到 '{save_base}'")
            else:
                save_file = save_base + ".json"
                with open(save_file, "w") as f:
                    json.dump(masks, f)
                print(f"  -> 生成了 {len(masks)} 个掩码，保存到 '{save_file}'")
            
            total_processed += 1
            
            # 清理内存和显存
            del image, masks
            gc.collect()
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
                print(f"  -> 已清理显存，当前显存使用: {torch.cuda.memory_allocated() / 1024**2:.1f} MB")
        
        # 处理完一个文件夹后的深度清理
        print(f"完成文件夹 '{folder_path}' 的处理，进行深度内存清理...")
        gc.collect()
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
            torch.cuda.synchronize()
            print(f"深度清理后显存使用: {torch.cuda.memory_allocated() / 1024**2:.1f} MB")
            
    print(f"\n处理完成！总共处理了 {total_processed} 张图片。")
    
    # 最终清理
    print("最终内存和显存清理...")
    gc.collect()
    if torch.cuda.is_available():
        torch.cuda.empty_cache()
        torch.cuda.synchronize()
        print(f"最终清理后显存使用: {torch.cuda.memory_allocated() / 1024**2:.1f} MB")


if __name__ == "__main__":
    args = parser.parse_args()
    main(args)
