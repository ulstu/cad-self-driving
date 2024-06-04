class TableHelper:
    """Класс для операций с табличными данными"""

    def remove_nan_values(self, lst: list) -> list:
        """Удаляет из списка нечисловые значения"""
        result = []
        for item in lst:
            try:
                float(item)
                result.append(item)
            except ValueError:
                continue
        return result

    def is_digit(self, item):
        try:
            float(item)
            return True
        except ValueError:
            return False

    def prepare_data_y_for_x(self, x_data: list, y_data: list):
        """Подгоняет данные при разной размерности x и y"""
        x_return = []
        y_return = []
        for x, y in zip(x_data, y_data):
            if self.is_digit(x):
                x_return.append(x)
                y_return.append(y)
        return x_return, y_return

