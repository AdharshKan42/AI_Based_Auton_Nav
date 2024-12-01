import os
import random
import shutil

def split_train_val(dataset_dir, val_ratio=0.2, seed=42):
    """
    Splits the dataset into train and validation sets.

    Args:
        dataset_dir (str): Path to the dataset directory containing `images/train` and `labels/train`.
        val_ratio (float): Proportion of data to use for validation (default: 0.2).
        seed (int): Random seed for reproducibility (default: 42).
    """
    # Set random seed for reproducibility
    random.seed(seed)

    # Directories for images and labels
    images_dir = os.path.join(dataset_dir, "images", "train")
    labels_dir = os.path.join(dataset_dir, "labels", "train")

    # Output directories
    # train_images_dir = os.path.join(dataset_dir, "images", "train")
    val_images_dir = os.path.join(dataset_dir, "images", "val")
    # train_labels_dir = os.path.join(dataset_dir, "labels", "train")
    val_labels_dir = os.path.join(dataset_dir, "labels", "val")

    # Create validation directories
    os.makedirs(val_images_dir, exist_ok=True)
    os.makedirs(val_labels_dir, exist_ok=True)

    # Get sorted lists of files
    image_files = sorted(os.listdir(images_dir))
    label_files = sorted(os.listdir(labels_dir))

    # Ensure matching image and label file counts
    assert len(image_files) == len(label_files), "Number of images and labels must match."

    # Pair images and labels
    data_pairs = list(zip(image_files, label_files))

    # Shuffle and split
    random.shuffle(data_pairs)
    val_size = int(len(data_pairs) * val_ratio)
    val_pairs = data_pairs[:val_size]
    train_pairs = data_pairs[val_size:]

    # Move files to validation folder
    for img_file, lbl_file in val_pairs:
        shutil.move(os.path.join(images_dir, img_file), os.path.join(val_images_dir, img_file))
        shutil.move(os.path.join(labels_dir, lbl_file), os.path.join(val_labels_dir, lbl_file))

    print(f"Dataset split complete.")
    print(f"Train size: {len(train_pairs)}, Validation size: {len(val_pairs)}")
