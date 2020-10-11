import numpy as np


def matMul(a, b):
    rowLen = len(a)
    colLen = len(b[0])
    # print(rowLen, colLen)
    for i in range(len(a)):
        for j in range(len(a[0])):
            a[i][j] = str(a[i][j])
    for i in range(len(b)):
        for j in range(len(b[0])):
            b[i][j] = str(b[i][j])

    mat1 = divideMat(a)
    mat2 = divideMat(b)
    # print(mat1)
    # print(mat2)

    answer = [[[] for j in range(len(b[0]))] for i in range(rowLen)]
    for i in range(rowLen):
        tar1 = mat1[i]
        for j in range(colLen):
            tar2 = [mat2[k][j] for k in range(rowLen)]
            answer[i][j] = merge(tar1, tar2)
    # show(answer)
    rt = recoverMat(answer)
    return rt


def divideMat(mat):
    rowLen = len(mat)
    colLen = len(mat[0])
    rt = [[[] for j in range(colLen)] for i in range(rowLen)]
    for i in range(rowLen):
        for j in range(colLen):
            t = mat[i][j]
            start = 1
            while True:
                start = t.find('-', start)
                if start == -1:
                    break
                t = t[0:start] + '+' + t[start:]
                start += 2
            for target in t.split('+'):
                k = 0
                c = 1
                if target[k] == '-':
                    k += 1
                    c *= -1
                n = ''
                while '0' <= target[k] <= '9':
                    n = n + target[k]
                    k += 1
                    if k >= len(target):
                        break
                if n == '':
                    n = '1'
                c = c*int(n)
                rt[i][j].append({'coe':c, 'var':target[k:]})
    return rt


def recoverMat(mat):
    rt = []
    for i in mat:
        row = []
        for j in i:
            s = ''
            for k in j:
                coe = k['coe']
                var = k['var']
                if coe < 0:
                    if coe == -1 and var:
                        s = s + '-' + var
                    elif coe == -1 and not var:
                        s = s + '-1'
                    else:
                        s = s + str(coe) + var
                elif coe == 0:
                    s = s + '+' + str(coe)
                else:
                    if coe == 1 and var:
                        s = s + '+' + var
                    elif coe == 1 and not var:
                        s = s + '+1'
                    else:
                        s = s + '+' + str(coe) + var
            if s[0] == '+':
                s = s[1:]
            row.append(s)
        rt.append(row)
    return rt


def merge(m1, m2):
    rt = []
    # print(m1)
    # print(m2)
    for i in range(len(m1)):
        tar1 = m1[i]
        tar2 = m2[i]
        coe = 0
        var = ''
        for j in tar1:
            for k in tar2:
                coe = j['coe'] * k['coe']
                if not coe:
                    continue
                var = j['var'] + k['var']
                rt.append({'coe': coe, 'var': var})
    if not rt:
        rt.append({'coe': 0, 'var': ''})
    return rt


def show(mat):
    for i in mat:
        print("[\t", end='')
        for j in range(len(i)):
            print(i[j], '\t', end='')
        print("]")


if __name__ == '__main__':
    Rx = [['1', '0',    '0'],
          ['0', 'cos1', '-sin1'],
          ['0', 'sin1', 'cos1']]

    Ry = [['cos2',  '0', 'sin2'],
          ['0',     '1', '0'],
          ['-sin2', '0', 'cos2']]

    Rz = [['cos3', '-sin3', '0'],
          ['sin3', 'cos3',  '0'],
          ['0',    '0',     '1']]
    ex = [['1'], ['0'], ['0']]
    R1 = matMul(Rz, Ry)
    show(R1)
    R2 = matMul(R1, Rx)
    show(R2)
    R3 = matMul(R2, ex)
    show(R3)

