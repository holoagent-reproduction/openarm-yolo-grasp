#!/usr/bin/env python3
"""只负责 YOLOE 推理的 HTTP 服务；不包含任何机械臂控制接口。"""
import argparse
import base64
import json
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import torch
from fastapi import FastAPI, File, Form, HTTPException, UploadFile
from ultralytics import YOLOE


class YoloeRuntime:
    def __init__(self, model_path: str, device: str):
        self.device = device if device != "auto" else ("cuda:0" if torch.cuda.is_available() else "cpu")
        self.model = YOLOE(model_path)
        self.prompts = ()
        self.lock = threading.Lock()

    def infer(self, encoded: bytes, prompts: list[str], conf: float, iou: float, imgsz: int):
        image = cv2.imdecode(np.frombuffer(encoded, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("无法解码输入图像")
        prompts_key = tuple(prompts)
        with self.lock:
            if prompts_key != self.prompts:
                self.model.predictor = None
                self.model.set_classes(prompts, self.model.get_text_pe(prompts))
                self.prompts = prompts_key
            started = time.perf_counter()
            results = self.model.track(
                source=image, device=self.device, conf=conf, iou=iou,
                imgsz=imgsz, persist=True, verbose=False,
            )
            elapsed_ms = (time.perf_counter() - started) * 1000.0
        output = []
        if not results:
            return output, elapsed_ms, image.shape[1], image.shape[0]
        result = results[0]
        if result.boxes is None:
            return output, elapsed_ms, image.shape[1], image.shape[0]
        for index, box in enumerate(result.boxes):
            xyxy = [int(v) for v in box.xyxy[0].detach().cpu().tolist()]
            class_id = int(box.cls[0].detach().cpu())
            track_id = -1
            if box.id is not None:
                track_id = int(box.id[0].detach().cpu())
            mask_b64 = ""
            if result.masks is not None and index < len(result.masks.data):
                mask = result.masks.data[index].detach().cpu().numpy()
                mask = cv2.resize(mask, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_NEAREST)
                ok, mask_png = cv2.imencode(".png", (mask > 0.5).astype(np.uint8) * 255)
                if ok:
                    mask_b64 = base64.b64encode(mask_png.tobytes()).decode("ascii")
            output.append({
                "class_id": class_id,
                "class_name": result.names.get(class_id, str(class_id)),
                "confidence": float(box.conf[0].detach().cpu()),
                "track_id": track_id,
                "bbox_xyxy": xyxy,
                "mask_png_base64": mask_b64,
            })
        return output, elapsed_ms, image.shape[1], image.shape[0]


def create_app(runtime: YoloeRuntime):
    app = FastAPI(title="OpenArm YOLOE 推理服务", version="0.1.0")

    @app.get("/health")
    def health():
        return {"ok": True, "device": runtime.device, "model": type(runtime.model).__name__}

    @app.post("/infer")
    async def infer(
        image: UploadFile = File(...),
        prompts: str = Form(...),
        confidence: float = Form(0.40),
        iou: float = Form(0.60),
        image_size: int = Form(640),
    ):
        try:
            prompt_list = json.loads(prompts)
            if not isinstance(prompt_list, list) or not prompt_list or not all(isinstance(v, str) for v in prompt_list):
                raise ValueError("prompts 必须是非空字符串列表")
            detections, elapsed_ms, width, height = runtime.infer(
                await image.read(), prompt_list, confidence, iou, image_size)
            return {"width": width, "height": height, "elapsed_ms": elapsed_ms, "detections": detections}
        except (ValueError, json.JSONDecodeError) as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        except Exception as exc:
            raise HTTPException(status_code=500, detail=f"推理失败：{exc}") from exc

    return app


def main():
    parser = argparse.ArgumentParser(description="启动 OpenArm YOLOE HTTP 推理服务")
    parser.add_argument("--model", default="yoloe-11s-seg.pt", help="YOLOE 文本提示分割模型")
    parser.add_argument("--device", default="auto", help="auto、cuda:0 或 cpu")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    import uvicorn
    uvicorn.run(create_app(YoloeRuntime(args.model, args.device)), host=args.host, port=args.port)


if __name__ == "__main__":
    main()
