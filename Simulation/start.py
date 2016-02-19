from bottle import route, run, jinja2_template as template, error, static_file


@route("/")
def index():
    return template("index.html")

@route("/public/<filename:path>")
def static_url(filename):
    return static_file(filename, root="./public")

@error(404)
def error404(error):
    return 'Nothing here, sorry'

run(host='localhost', port=8080, debug=True)