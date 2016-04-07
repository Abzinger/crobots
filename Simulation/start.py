from bottle import route, run, jinja2_template as template, error, static_file, response
import gridTools.fileReader as fr

routes = fr.getRouteList()

@route("/")
def index():
    return template("index.html")

@route("/public/<filename:path>")
def static_url(filename):
    return static_file(filename, root="./public")

@route("/Route/GetRouteList")
def routeList():
    response.content_type = 'application/json'
    return fr.getJSONRouteList()


@route("/Route/<routeNr:int>/Layout")
def layout(routeNr):
    response.content_type = 'application/json'
    return fr.getParkingLayout(fr.getRoute(routes[routeNr]))



@error(404)
def error404(error):
    return 'Nothing here, sorry'

run(host='localhost', port=8080, debug=True)