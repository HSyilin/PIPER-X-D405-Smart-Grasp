# Segmentation models

Place the externally trained model here as `blue_block_seg.pt`, together with
`model_metadata.yaml` and `sha256.txt`. Select it explicitly with
`detector_backend:=yolo_seg`; missing weights are a startup error.
