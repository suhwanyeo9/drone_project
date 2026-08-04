import numpy as np
from collections import deque

A = np.load('/home/godns/Downloads/arena01.npy')
H, W = A.shape
RES = 0.05
R0, R1, C0, C1 = 48, 107, 43, 103

def fit_wall(pts, label):
    if len(pts) < 5:
        print(f'{label}: 점 부족 ({len(pts)})')
        return
    p = np.array(pts, float)
    s, b = np.polyfit(p[:,0], p[:,1], 1)
    resid = p[:,1] - (s*p[:,0] + b)
    print(f'{label}: {np.degrees(np.arctan(s)):+.2f} deg   점 {len(pts):>3}개   잔차 {resid.std():.2f}px')

L=[]; R=[]; T=[]; B=[]
for r in range(R0, R1+1):
    lo = max(0, C0-5)
    w = np.where(A[r, lo:C0+8] == 0)[0]
    if len(w): L.append((r, lo + w[0]))
    w = np.where(A[r, C1-7:min(W, C1+6)] == 0)[0]
    if len(w): R.append((r, C1-7 + w[-1]))
for c in range(C0, C1+1):
    lo = max(0, R0-5)
    w = np.where(A[lo:R0+8, c] == 0)[0]
    if len(w): T.append((c, lo + w[0]))
    w = np.where(A[R1-7:min(H, R1+6), c] == 0)[0]
    if len(w): B.append((c, R1-7 + w[-1]))

print('=== 실제 벽 픽셀 기준 각도 ===')
fit_wall(L,'좌벽'); fit_wall(R,'우벽'); fit_wall(T,'상벽'); fit_wall(B,'하벽')

M = 3
ir0, ir1, ic0, ic1 = R0+M, R1-M, C0+M, C1-M
sub = A[ir0:ir1+1, ic0:ic1+1]
obst = (sub != 254)
h, w = obst.shape
lab = np.zeros((h, w), int); n = 0
for i in range(h):
    for j in range(w):
        if obst[i,j] and lab[i,j] == 0:
            n += 1
            q = deque([(i,j)]); lab[i,j] = n
            while q:
                y,x = q.popleft()
                for dy,dx in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                    ny,nx = y+dy, x+dx
                    if 0<=ny<h and 0<=nx<w and obst[ny,nx] and lab[ny,nx]==0:
                        lab[ny,nx] = n; q.append((ny,nx))

cr = (R0+R1)/2.0; cc = (C0+C1)/2.0
print('\n=== 내부 장애물 (방 중심 기준, m) ===')
print(f'{"id":>3} {"px":>4} {"x":>7} {"y":>7} {"가로":>6} {"세로":>6}')
big = 0
for k in range(1, n+1):
    ys, xs = np.where(lab == k)
    if len(ys) < 3: continue
    big += 1
    rr = ys + ir0; ccs = xs + ic0
    print(f'{big:>3} {len(ys):>4} '
          f'{(ccs.mean()-cc)*RES:>7.2f} {(cr-rr.mean())*RES:>7.2f} '
          f'{(ccs.max()-ccs.min()+1)*RES:>6.2f} {(rr.max()-rr.min()+1)*RES:>6.2f}')
print(f'\n총 클러스터 {n}개 중 3px 이상 {big}개')
