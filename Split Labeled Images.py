import os
import shutil
import random

# Paths
img_dir = r'W:/Documents Storage/UTEM/Now/Y3S1/BITU 3973 FYP/dataset/images/img'
label_dir = r'W:/Documents Storage/UTEM/Now/Y3S1/BITU 3973 FYP/dataset/labels/lbl'

# Output paths
train_img_dir = os.path.join(img_dir, 'train')
val_img_dir = os.path.join(img_dir, 'val')
train_label_dir = os.path.join(label_dir, 'train')
val_label_dir = os.path.join(label_dir, 'val')

# Make folders
for d in [train_img_dir, val_img_dir, train_label_dir, val_label_dir]:
    os.makedirs(d, exist_ok=True)

# Match image-label pairs
image_files = [f for f in os.listdir(img_dir) if f.lower().endswith(('.jpg', '.png'))]
labeled_images = []

for img in image_files:
    label_name = os.path.splitext(img)[0] + ".txt"
    label_path = os.path.join(label_dir, label_name)
    if os.path.exists(label_path):
        labeled_images.append(img)

# Shuffle and split
random.shuffle(labeled_images)
split_idx = int(len(labeled_images) * 0.8)
train_imgs = labeled_images[:split_idx]
val_imgs = labeled_images[split_idx:]

# Move function
def move_files(image_list, img_dest, label_dest):
    for img_name in image_list:
        label_name = os.path.splitext(img_name)[0] + ".txt"
        img_src = os.path.join(img_dir, img_name)
        label_src = os.path.join(label_dir, label_name)
        img_dst = os.path.join(img_dest, img_name)
        label_dst = os.path.join(label_dest, label_name)

        # Move image
        shutil.move(img_src, img_dst)

        # Only move label if it exists
        if os.path.exists(label_src):
            shutil.move(label_src, label_dst)
        else:
            print(f"⚠️ Warning: Label file not found for {img_name} ({label_name})")

move_files(train_imgs, train_img_dir, train_label_dir)
move_files(val_imgs, val_img_dir, val_label_dir)

print(f"✅ Done! {len(train_imgs)} images in train/, {len(val_imgs)} in val/")
