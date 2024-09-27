const PORT = 9876;
const HOST = '192.168.169.1';
const fs = require("fs");
const dgram = require('dgram');
const message = Buffer.from('capture');
let timeout = 0;
let client = dgram.createSocket('udp4');
let bufArray = [];
let jpegLength = 0;
fs.writeFileSync("./tmp.pcm", "");
// const Speaker = require('speaker');

// // 创建一个新的Speaker实例
// const speaker = new Speaker({
//     channels: 1,          // 2 通道（立体声）
//     bitDepth: 16,         // 每个样本的位数
//     sampleRate: 32000,    // 样本率
//     float: true,          // 不使用浮点数
//     signed: true,
//     samplesPerFrame:8
// });
function writeBuffer (buf)
{
    return new Promise(resolve =>
    { 
        speaker.write(buf, () =>
        {
            console.log("done")
        });
        resolve();  
    });
}
client.addListener("message", async (msg) =>
{
    let jpegEnd = false;
    if (msg.byteLength < 64)
    {
        let str = msg.toString();
        if (str.indexOf("len:") > -1)
        {
            jpegLength = str.split("len:")[1] * 1;
            console.log("jpegLength:", jpegLength);
        }
        bufArray = [];
        capture();
        return;
    }
    fs.appendFileSync("./tmp.pcm", msg);
    // for (let i = 0; i < msg.byteLength; i++)
    // {
    //     bufArray.push(msg[i]);
    // }
    // if (bufArray.length >= 8000)
    // {
    //     console.log("msg:", bufArray.length);
    //     await writeBuffer(Buffer.from(bufArray));
    // }
    capture();
    return;
    for (let i = 0; i < msg.byteLength-1; i++)
    {
        if (msg[i] == 0xff && msg[i+1] == 0xd8)
        {
            console.log("JPEG HEAD FOUND:", i);
            bufArray = [];
            break;
        }
    }
    for (let i = 0; i < msg.byteLength; i++)
    {
        bufArray.push(msg[i]);
    }
    for (let i = 0; i < msg.byteLength-1; i++)
    {
        if (msg[i] == 0xff && msg[i+1] == 0xd9)
        {
            console.log("JPEG END FOUND:", i, msg.byteLength, bufArray.length);
            if (jpegLength == bufArray.length)
            {
                fs.writeFileSync("./tmp.jpg", Buffer.from(bufArray));
            }
            else
            {
                console.log("wrong jpeg length");
            }
            // console.timeEnd("capture");
            bufArray = [];
            jpegEnd = true;
            break;
        }
    }
    // if (bufArray.length > 0)
    {
        capture();
    }
    // if (jpegEnd)
    // {
    //     capture();
    // }
});
capture();
function capture ()
{
    clearTimeout(timeout);
    client.send(message, PORT, HOST, function(err, bytesCount) {
        if (err) throw err;
        // console.log('UDP message sent to ' + HOST + ':' + PORT, bytesCount);
        // console.time("capture");
        timeout = setTimeout(() =>
        {
            // console.timeEnd("capture");
            console.log("TIMEOUT!!!!!\n");
            capture();
        }, 1000);
    });
}
function request ()
{
    client.send(Buffer.from(' '), PORT, HOST, function(err, bytesCount) {
        if (err) throw err;
    });
}