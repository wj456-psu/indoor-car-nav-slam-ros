import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import convolve


def read_image(file: str):
    with open(file, "rb") as f:
        return plt.imread(f)


def main():
    files: list[str] = [
        "recmap1_edited.pgm",
        "recmap2_edited.pgm",
        "recmap4_edited.pgm",
        "recmap3_edited.pgm",
        "recmap5_edited.pgm",
        "recmap6_edited.pgm",
    ]
    images = [read_image(file) for file in files]
    bin_images = [np.where(image > 100, 1.0, 0.0) for image in images]
    obstacle_densities = [
        convolve(image, np.ones((2, 2)) / 4, mode="constant", cval=0)
        for image in bin_images
    ]
    for densities in obstacle_densities:
        _, freq_val = np.unique(densities, return_counts=True)
        probs = freq_val / densities.size
        print(-np.sum(probs * np.log2(probs)))

    fig = plt.figure()
    rows, cols = 1, len(images)

    for i, image in enumerate(images, start=1):
        fig.add_subplot(rows, cols, i)
        plt.imshow(image, cmap="gray")
        plt.title(f"Image {i} ({image.shape[1]}, {image.shape[0]})")

    plt.show()


if __name__ == "__main__":
    main()
