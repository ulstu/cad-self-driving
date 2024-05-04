import cherrypy
from config import conf
import config
from main import Main


@cherrypy.expose
class CherryApp:
    """Основной класс для бэкэнда веб-приложения"""

    @cherrypy.tools.json_out()
    def GET(self):
        return {"data": Main().response}

    @cherrypy.tools.json_out()
    @cherrypy.tools.json_in()
    def POST(self):
        """Метод рассчёта всех параметров и возврата json-файла"""
        params = cherrypy.request.json
        if not params:
            return {'error': 'EMPTY PARAMS'}
        return {"data": Main(params).response}

    @cherrypy.tools.json_out()
    @cherrypy.tools.json_in()
    def OPTIONS(self):
        cherrypy_cors.preflight(allowed_methods=['GET', 'POST'])


def CORS():
    cherrypy.response.headers["Access-Control-Allow-Origin"] = "*"


if __name__ == '__main__':
    import cherrypy_cors

    cherrypy_cors.install()
    cherrypy.tools.CORS = cherrypy.Tool('before_finalize', CORS)
    cherrypy.quickstart(CherryApp(), '/', conf)
