#pragma once

// RUNコマンド: READY(補間完了後)からRUNへ遷移
void cmd_run(void* dora_context);

// tick時のRUN処理
void tick_run(void* dora_context);
