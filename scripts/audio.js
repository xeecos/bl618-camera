const fs = require('fs');
const Speaker = require('speaker');

// 创建一个新的Speaker实例
const speaker = new Speaker({
    channels: 1,          // 2 通道（立体声）
    bitDepth: 16,         // 每个样本的位数
    sampleRate: 32000,    // 样本率
    float: false,          // 不使用浮点数
    signed: true,
    samplesPerFrame: 1000
});

// 打开音频文件并播放
fs.createReadStream('./tmp.pcm').pipe(speaker);
