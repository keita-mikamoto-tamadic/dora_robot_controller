#pragma once

// READYコマンド: 初期位置へ補間移動
void cmd_ready(void* dora_context);

// tick時の補間処理
void tick_interpolation(void* dora_context);

// 補間開始
void start_interpolation();

// 補間計算
double interpolate(double start, double target, double t);
