from pathlib import Path

import cv2
import numpy as np
import matplotlib.pyplot as plt

caminho_img = Path(__file__).resolve().parents[1] / 'dog.jpg'
img_bgr = cv2.imread(str(caminho_img))
if img_bgr is None:
    raise FileNotFoundError(f"Nao foi possivel carregar a imagem: {caminho_img}")

img_bgr = img_bgr.astype(np.float64)
img = 0.114 * img_bgr[:, :, 0] + 0.587 * img_bgr[:, :, 1] + 0.299 * img_bgr[:, :, 2]

def gaussian_kernel(size=5, sigma=1.4):
    ax = np.arange(-(size // 2), size // 2 + 1)
    xx, yy = np.meshgrid(ax, ax)
    kernel = np.exp(-(xx**2 + yy**2) / (2 * sigma**2))
    return kernel / kernel.sum()

def convolve(image, kernel):
    pad = kernel.shape[0] // 2
    padded = np.pad(image, pad, mode='reflect')
    output = np.zeros_like(image)
    for i in range(image.shape[0]):
        for j in range(image.shape[1]):
            output[i, j] = np.sum(
                padded[i:i+kernel.shape[0], j:j+kernel.shape[1]] * kernel
            )
    return output

blurred = convolve(img, gaussian_kernel())

def sobel(image):
    Kx = np.array([[-1, 0, 1],
                   [-2, 0, 2],
                   [-1, 0, 1]], dtype=np.float64)

    Ky = np.array([[-1, -2, -1],
                   [ 0,  0,  0],
                   [ 1,  2,  1]], dtype=np.float64)

    Gx = convolve(image, Kx)
    Gy = convolve(image, Ky)

    magnitude = np.sqrt(Gx**2 + Gy**2)
    maximo = magnitude.max()
    if maximo == 0:
        return magnitude

    return magnitude / maximo * 255

edges = sobel(blurred)

threshold = 80
binary = (edges > threshold).astype(np.uint8) * 255

fig, axes = plt.subplots(1, 3, figsize=(15, 5))
axes[0].imshow(img, cmap='gray')
axes[0].set_title('1. Escala de Cinza')
axes[1].imshow(edges, cmap='gray')
axes[1].set_title('2. Bordas (Sobel)')
axes[2].imshow(binary, cmap='gray')
axes[2].set_title('3. Limiarizacao')

for ax in axes:
    ax.axis('off')

plt.tight_layout()
plt.savefig(Path(__file__).with_name('resultado_visao.png'), dpi=150)
plt.show()

points = np.argwhere(binary > 0)

print(f"Total de pontos de borda: {len(points)}")
print(f"Visualizacao salva em: {Path(__file__).with_name('resultado_visao.png')}")