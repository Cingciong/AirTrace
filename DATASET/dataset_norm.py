import os
import pandas as pd
import numpy as np
import shutil

dataset_dir = "Dataset"
labels_dir = os.path.join(dataset_dir, "labels")
images_dir = os.path.join(dataset_dir, "images")

output_dir = "Dataset_balanced"
output_images = os.path.join(output_dir, "images")
output_labels = os.path.join(output_dir, "labels")

os.makedirs(output_labels, exist_ok=True)
os.makedirs(os.path.join(output_images, "train"), exist_ok=True)
os.makedirs(os.path.join(output_images, "val"), exist_ok=True)
os.makedirs(os.path.join(output_images, "test"), exist_ok=True)

train_df = pd.read_csv(os.path.join(labels_dir, "train.csv"))
val_df = pd.read_csv(os.path.join(labels_dir, "val.csv"))
test_df = pd.read_csv(os.path.join(labels_dir, "test.csv"))

all_df = pd.concat([train_df, val_df, test_df], ignore_index=True)

all_df = all_df[all_df["alt"] >= 2.0]

num_bins = 100
max_per_bin = 500

counts, bins = np.histogram(all_df["alt"], bins=num_bins)
balanced_rows = []

for i in range(num_bins):
    bin_mask = (all_df["alt"] >= bins[i]) & (all_df["alt"] < bins[i+1])
    bin_df = all_df[bin_mask]
    if len(bin_df) > max_per_bin:
        bin_df = bin_df.sample(n=max_per_bin, random_state=42)
    balanced_rows.append(bin_df)

balanced_df = pd.concat(balanced_rows, ignore_index=True)

train_end = int(0.8 * len(balanced_df))
val_end = train_end + int(0.5 * (len(balanced_df) - train_end))

train_df = balanced_df.iloc[:train_end]
val_df = balanced_df.iloc[train_end:val_end]
test_df = balanced_df.iloc[val_end:]

train_df.to_csv(os.path.join(output_labels, "train.csv"), index=False)
val_df.to_csv(os.path.join(output_labels, "val.csv"), index=False)
test_df.to_csv(os.path.join(output_labels, "test.csv"), index=False)

def copy_images(df, split):
    for img in df["img"]:
        src_train = os.path.join(images_dir, "train", img)
        src_val = os.path.join(images_dir, "val", img)
        src_test = os.path.join(images_dir, "test", img)

        if os.path.exists(src_train):
            src = src_train
        elif os.path.exists(src_val):
            src = src_val
        else:
            src = src_test

        dst = os.path.join(output_images, split, img)
        shutil.copy(src, dst)

copy_images(train_df, "train")
copy_images(val_df, "val")
copy_images(test_df, "test")

for split in ["train", "val", "test"]:
    n_csv = len(pd.read_csv(os.path.join(output_labels, f"{split}.csv")))
    n_img = len(os.listdir(os.path.join(output_images, split)))
    print(split, "CSV:", n_csv, "images:", n_img)