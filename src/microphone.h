#pragma once

void microphone_init();
uint8_t *audio_data();
int audio_status();
void audio_set_status(int status);