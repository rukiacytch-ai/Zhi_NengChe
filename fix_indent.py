"""Add debug printf to recording section in ISR."""
with open('user/cpu0_main.c', 'rb') as f:
    data = f.read()

sep = b'\r\n'
lines = data.split(sep)

# Find the recording block and insert debug code
# Looking for lines containing 'if (is_recording)' and the following '{'
for i, line in enumerate(lines):
    if b'if (is_recording) //' in line and b'{' in lines[i+1]:
        indent12 = b' ' * 12
        indent16 = b' ' * 16

        # Build the replacement block
        replacement = [
            indent12 + b'if (is_recording) // ' + '如果正在记录模式'.encode('gbk'),
            indent12 + b'{',
            indent16 + b'static uint16 dbg_cnt = 0;',
            indent16 + b'dbg_cnt++;',
            indent16 + b'if (dbg_cnt >= 500) // ' + '每1秒打印一次编码器值'.encode('gbk'),
            indent16 + b'{',
            indent16 + b'    dbg_cnt = 0;',
            indent16 + b'    printf(' + '"ENC: R=%.2f L=%.2f Car=%.2f cm\\r\\n"'.encode('gbk') + b', encoder_right_loc, encoder_left_loc, Car_Go_Location);',
            indent16 + b'}',
            indent16 + b'if (Car_Go_Location >= Get_Dot_Loc) // ' + '如果小车位置大于等于打点距离'.encode('gbk'),
        ]

        # Remove the old 'if (is_recording)' line and '{' line
        # Then insert the replacement
        lines[i:i+2] = replacement
        break

with open('user/cpu0_main.c', 'wb') as f:
    f.write(sep.join(lines))

print('Debug printf added to ISR recording section.')
