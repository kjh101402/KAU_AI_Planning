x = 3

with open('test.txt', 'w') as f:
    lines = []
    lines.append(f'line {x+1}\n\n')
    lines.append(f'line {x+2}\n')
    
    f.writelines(lines)