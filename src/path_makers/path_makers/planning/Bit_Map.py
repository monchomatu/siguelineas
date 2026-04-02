import cv2
import numpy as np
import matplotlib.pyplot as plt


class BitmapMap:
    def __init__(self, png_path: str, world_size: float = 16.0):
        """
        Args:
            png_path: path al bitmap del mapa
            world_size: tamaño del mapa en metros (asume mapa cuadrado)
        """
        self.world_size = world_size
        self.half_size = world_size / 2.0

        # --- Leer imagen ---
        img = cv2.imread(png_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise RuntimeError(f"No se pudo cargar el mapa: {png_path}")

        self.height, self.width = img.shape

        # --- Normalizar y binarizar ---
        img = img.astype(np.float32) / 255.0
        self.occ_grid = np.zeros_like(img, dtype=np.uint8)
        self.occ_grid[img < 0.5] = 1

        # --- Resolución ---
        self.m_per_px = self.world_size / self.width

    # =====================================================
    # Conversión coordenadas
    # =====================================================

    def world_to_pixel(self, x: float, y: float):
        px = int((x + self.half_size) / self.world_size * self.width)
        py = int((self.half_size - y) / self.world_size * self.height)
        return px, py

    def pixel_to_world(self, px: int, py: int):
        x = (px / self.width) * self.world_size - self.half_size
        y = self.half_size - (py / self.height) * self.world_size
        return x, y

    # =====================================================
    # Consultas
    # =====================================================

    def in_bounds(self, x: float, y: float) -> bool:
        return (
            -self.half_size <= x <= self.half_size and
            -self.half_size <= y <= self.half_size
        )

    def is_occupied(self, x: float, y: float) -> bool:
        if not self.in_bounds(x, y):
            return True

        px, py = self.world_to_pixel(x, y)

        if px < 0 or px >= self.width or py < 0 or py >= self.height:
            return True

        return self.occ_grid[py, px] == 1

    #Inflar
    def inflate(self, robot_radius: float):
        """
        Infla los obstáculos según el radio del robot.

        Args:
            robot_radius: radio del robot en metros
        """
        # Radio en píxeles
        radius_px = int(np.ceil(robot_radius / self.m_per_px))

        if radius_px <= 0:
            return

        kernel_size = 2 * radius_px + 1
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (kernel_size, kernel_size)
        )

        inflated = cv2.dilate(self.occ_grid, kernel)

        self.occ_grid = np.clip(inflated, 0, 1).astype(np.uint8)

    def collision_free(self, p1, p2, step=None) -> bool:
        """
        Verifica si el segmento p1 -> p2 está libre de colisiones
        p1, p2: (x, y) en coordenadas mundo
        """
        x1, y1 = p1
        x2, y2 = p2

        dist = np.hypot(x2 - x1, y2 - y1)

        # Tamaño de muestreo (por defecto: mitad de un pixel)
        if step is None:
            step = self.m_per_px * 0.5

        n_samples = int(dist / step)

        for i in range(n_samples + 1):
            t = i / max(n_samples, 1)
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)

            if self.is_occupied(x, y):
                return False

        return True
      
    def debug_values(self):
        unique, counts = np.unique(self.occ_grid, return_counts=True)
        return dict(zip(unique.tolist(), counts.tolist()))

    # =====================================================
    # Visualización
    # =====================================================

    def plot(self):
        plt.figure(figsize=(6, 6))
        plt.imshow(
            self.occ_grid,
            cmap="gray_r",   # ← importante
            origin="upper",
            extent=[
                -self.half_size,
                self.half_size,
                -self.half_size,
                self.half_size,
            ]
        )

        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.title("Bitmap Map (ocupado = negro)")
        plt.grid(False)
        plt.show()
    
    def give_map(self):
        return self.occ_grid, self.half_size
