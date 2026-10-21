# Elevator checkpoint

`best.pt` is included with the owner's authorization. On 2026-10-20, the owner
confirmed personally training it in response to the request for permission to
include it in this repository. This records permission for this repository;
no general public redistribution license was specified. The manifest records
`license_status: authorized` and the scope of that confirmation.

The source training code identifies the labels explicitly, and safe checkpoint
inspection (`torch.load(..., weights_only=True)`) confirmed:

- MobileNetV3 Small; final classifier `Linear(..., 2)`.
- `classes = [available, unavailable]`, `image_size = 224`.
- Epoch 1, recorded validation accuracy 0.9883720930232558. This is training
  metadata, not evidence of performance on this robot or a basis for class order.
- 6,199,465 bytes; SHA256 is recorded in the manifest.
- CPU strict state-dict loading and a `[1,3,224,224]` forward pass succeeded with
  the source checkpoint on the development host.

Only the checkpoint was copied from the source package; virtual environments,
training data and training history were excluded. The loader resolves paths
from this repository and verifies SHA256 before loading. The checkpoint is
about 6 MB; ordinary Git is feasible for this single version. A narrow
`.gitignore` exception allows this specific checkpoint. For larger replacements
or many versions, use Git LFS according to the repository host's storage policy.

Runtime dependencies, installed in the **same Python environment as the UI**:
`torch`, `torchvision`, `Pillow`, `numpy`, `PyYAML`, plus the existing ROS 2 and Qt
packages. Install the CPU packages into the UI environment with:

```sh
venv/bin/python -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

No environment from Downloads has been reused or copied.

Preprocessing is full-frame RGB (ROS bgr8 is converted), PIL/torchvision bilinear
resize to 224×224, ToTensor, ImageNet normalization (mean .485/.456/.406,
std .229/.224/.225). There is no crop. Softmax confidence must be at least .85
for a known state; three distinct fresh `available` frames are required to begin
motion. This threshold is a conservative implementation choice, not calibrated
field performance. The classifier and motion gate both use `confidence_threshold`
from the manifest, validated at model load. Restart the UI after a configuration
change. The original source predictor defaults to .70; integration retains .85
until camera evidence supports recalibration.

To update: confirm usage rights, verify architecture/class order/preprocessing
against the new training code and checkpoint, replace the checkpoint, update
version/hash/size/license evidence, run offline tests and a controlled camera
validation, then perform the robot acceptance procedure in
`docs/elevator-test.md`. Never infer class order from `history.json` accuracy.
