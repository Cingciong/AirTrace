import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

dataset_dir = "Dataset_balanced/labels"
#dataset_dir = "Dataset/labels"

train_df = pd.read_csv(os.path.join(dataset_dir, "train.csv"))
val_df = pd.read_csv(os.path.join(dataset_dir, "val.csv"))
test_df = pd.read_csv(os.path.join(dataset_dir, "test.csv"))

all_df = pd.concat([train_df, val_df, test_df], ignore_index=True)

num_bins = 50

fig, axs = plt.subplots(2, 2, figsize=(12,8))

datasets = [
    ("All", all_df, axs[0,0]),
    ("Train", train_df, axs[0,1]),
    ("Val", val_df, axs[1,0]),
    ("Test", test_df, axs[1,1])
]

for title, df, ax in datasets:
    altitudes = df["alt"].values
    counts, bins = np.histogram(altitudes, bins=num_bins)
    bin_centers = (bins[:-1] + bins[1:]) / 2
    ax.bar(bin_centers, counts, width=(bins[1]-bins[0])*0.9)
    ax.set_title(title)
    ax.set_xlabel("Height relative to ground level H[m]")
    ax.set_ylabel("Number of samples N")
    ax.grid(axis='y', linestyle='--', alpha=0.5)

plt.tight_layout()
plt.show()