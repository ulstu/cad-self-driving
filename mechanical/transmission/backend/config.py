import os

import cherrypy

folder = os.path.abspath(os.curdir)
conf = {
    'global': {'server.socket_host': '0.0.0.0', 'server.socket_port': 8080},
    '/': {
        'cors.expose.on': True,
        'request.dispatch': cherrypy.dispatch.MethodDispatcher(),
        'tools.sessions.on': True,
        'tools.staticdir.root': os.path.abspath(os.getcwd()),
        'tools.response_headers.headers': [('Content-Type', 'text/plain')],
        'tools.encode.on': True,
        'tools.encode.encoding': 'utf-8',
        'tools.decode.on': True,
        'tools.CORS.on': True
    },
    '/static': {
        'tools.staticdir.on': True,
        'tools.staticdir.dir': os.path.join(folder, 'static'),
        'tools.staticdir.content_types': {'html': 'application/octet-stream'}
    }
}

BASE_DIR = os.path.abspath(os.path.curdir)
GRAPHICS_DIR = os.path.join(BASE_DIR, 'static', 'graphics')
if not os.path.exists(GRAPHICS_DIR):
    os.makedirs(GRAPHICS_DIR)
