from pandas import DataFrame


class JSONHelper:
    """Класс для преобразований данных в JSON"""

    def dataframes_to_dict(self, dataframes: list[DataFrame]) -> dict:
        """Преобразование листа датафреймов в JSON"""
        main_json = {}
        counter = 0  # На всякий случай, если у ДатаФрейма не будет имени
        for dataframe in dataframes:
            dataframe_json = dataframe.to_dict()
            if hasattr(dataframe, 'name'):
                main_json[dataframe.name] = {**dataframe_json}
            else:
                main_json[f'some_table_{counter}'] = {**dataframe_json}
                counter += 1
        return main_json
