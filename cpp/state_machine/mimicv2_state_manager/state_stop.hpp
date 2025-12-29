#pragma once

// STOPコマンド: NaN位置制御で現在位置保持（moteusドライバ内で制御）
void cmd_stop(void* dora_context);

// tick時のホールド処理（READY状態とSTOP状態で使用）
void tick_hold(void* dora_context);
