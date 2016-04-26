from bottle import (route, run, jinja2_template as template,
                    error, static_file, response, get)
import gridTools.fileReader as FileReader

routes = FileReader.get_route_list()


# Get index page
@route("/")
def index():
    return template("index.html")


# Route for static files
@route("/public/<file_name:path>")
def static_url(file_name):
    return static_file(file_name, root="./public")


# Get all the routes
@route("/Route/GetRouteList")
def route_list():
    response.content_type = 'application/json'
    return FileReader.get_json_route_list()


# Get the layout of the selected route
@route("/Route/<route_number:int>/Layout/<first:int>")
def layout(route_number, first):
    response.content_type = 'application/json'
    if first == 1:
        return FileReader.get_parking_layout(FileReader.get_route(routes[route_number]))
    else:
        return FileReader.get_parking_layout(FileReader.get_route(routes[route_number]), False)


# Get the instructions of the selected route
@route("/Route/<route_number:int>/Instructions")
def instructions(route_number):
    response.content_type = 'application/json'
    return FileReader.get_instructions(FileReader.get_route(routes[route_number]))


# Error page
@error(404)
def error404():
    return 'Nothing here, sorry'


# Favicon
@get('/favicon.ico')
def get_favicon():
    return static_file('favicon.ico', root="./public/assets")

run(host='localhost', port=8080, debug=True)
