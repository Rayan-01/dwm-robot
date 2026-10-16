"""Training, validation, inference and export commands for the poop detector."""

import argparse

from dwm_core.paths import model_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_subparsers(dest="action", required=True)
    train = actions.add_parser("train")
    train.add_argument("--data", required=True, help="Ultralytics dataset YAML")
    train.add_argument("--model", default="yolo26n.pt")
    train.add_argument("--epochs", type=int, default=150)
    train.add_argument("--patience", type=int, default=30)
    train.add_argument("--batch", type=int, default=-1)
    train.add_argument("--device", default="0")
    train.add_argument("--output", default="runs/detect")
    validate = actions.add_parser("validate")
    validate.add_argument("--data", required=True)
    validate.add_argument("--weights", default=str(model_path("best-v2.pt")))
    validate.add_argument("--device", default="0")
    predict = actions.add_parser("predict")
    predict.add_argument("--source", required=True)
    predict.add_argument("--weights", default=str(model_path("best-v2.pt")))
    predict.add_argument("--confidence", type=float, default=0.3)
    predict.add_argument("--device", default="0")
    predict.add_argument("--output", default="runs/predict")
    export = actions.add_parser("export")
    export.add_argument("--weights", default=str(model_path("best-v2.pt")))
    export.add_argument(
        "--format", choices=["onnx", "engine", "torchscript"], default="onnx"
    )
    export.add_argument("--device", default="0")
    args = parser.parse_args()
    from ultralytics import YOLO

    if args.action == "train":
        YOLO(args.model).train(
            data=args.data,
            epochs=args.epochs,
            patience=args.patience,
            batch=args.batch,
            imgsz=640,
            device=args.device,
            seed=42,
            deterministic=True,
            project=args.output,
            name="yolo26n_poop",
            exist_ok=False,
        )
    elif args.action == "validate":
        YOLO(args.weights).val(data=args.data, imgsz=640, device=args.device)
    elif args.action == "predict":
        YOLO(args.weights).predict(
            source=args.source,
            conf=args.confidence,
            imgsz=640,
            device=args.device,
            project=args.output,
            save=True,
        )
    else:
        # Export writes beside the weights; copy installed weights to a writable
        # directory and pass --weights when the installation is read-only.
        YOLO(args.weights).export(format=args.format, imgsz=640, device=args.device)


if __name__ == "__main__":
    main()
