from config import GRAPHICS_DIR


class GraphicHelper:
    """Хелпер для отрисовки графиков"""
    graphic_counter = 1

    @classmethod
    def save_graphic(cls, plt) -> None:
        """Сохраняет график в пнг"""
        plt.savefig(f'{GRAPHICS_DIR}/graph_{cls.graphic_counter}.png')
        cls.graphic_counter += 1

    @classmethod
    def reset_counter(cls):
        cls.graphic_counter = 1
