from bottle import route, run, jinja2_template as template, error, static_file, response
import gridTools.fileReader as fr

routes = fr.getRouteList()

# Get index page
@route("/")
def index():
    return template("index.html")

# Route for static files
@route("/public/<filename:path>")
def static_url(filename):
    return static_file(filename, root="./public")

# Get all the routes
@route("/Route/GetRouteList")
def routeList():
    response.content_type = 'application/json'
    return fr.getJSONRouteList()

# Get the layout of the selected route
@route("/Route/<routeNr:int>/Layout/<first:int>")
def layout(routeNr, first):
    response.content_type = 'application/json'
    if first == 1:
        return fr.getParkingLayout(fr.getRoute(routes[routeNr]))
    else:
        return fr.getParkingLayout(fr.getRoute(routes[routeNr]), False)

# Get the instructions of the selected route
@route("/Route/<routeNr:int>/Instructions")
def instructions(routeNr):
    response.content_type = 'application/json'
    return fr.getInstructions(fr.getRoute(routes[routeNr]))

# Error page
@error(404)
def error404(error):
    return 'Nothing here, sorry'

run(host='localhost', port=8080, debug=True)