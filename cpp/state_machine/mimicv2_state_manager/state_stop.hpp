#pragma once

// STOPコマンド: その場でサーボON、現在位置保持
void cmd_stop(void* dora_context);

// tick時のホールド処理
void tick_hold(void* dora_context);
