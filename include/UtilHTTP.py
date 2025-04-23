import requests

class HttpHeaders:
    CONTENT_TYPE = 'Content-Type'

class ContentTypes:
    APPLICATION_JSON = 'application/json'

class HttpMethod:
    GET = 'GET'
    POST = 'POST'
    PUT = 'PUT'
    DELETE = 'DELETE'
    PATCH = 'PATCH'
    OPTIONS = 'OPTIONS'
    HEAD = 'HEAD'
    TRACE = 'TRACE'
    CONNECT = 'CONNECT'
    
# headers = {
#     HttpHeaders.CONTENT_TYPE: ContentTypes.APPLICATION_JSON
# }

# response = requests.request(HttpMethod.GET, 'https://api.example.com/data', headers=headers)
# print(response.status_code)
