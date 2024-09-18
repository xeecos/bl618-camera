const http = require('http');
http.get('http://192.168.169.1/capture', res =>
{
    console.log(res);
});