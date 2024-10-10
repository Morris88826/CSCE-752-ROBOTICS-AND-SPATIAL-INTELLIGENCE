import numpy as np

def find_route(points):
    # Implement a path planning algorithm to find the shortest route to visit all targets
    n = len(points)
    dist = [[0] * n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            dist[i][j] = abs(points[i][0] - points[j][0]) + abs(points[i][1] - points[j][1])
    dist = np.array(dist)

    dp = [[float('inf')] * (1 << n) for _ in range(n)]
    dp = np.array(dp)
    dp[0][1] = 0

    for mask in range(1, 1 << n):
        for i in range(1, n): # 0 is the starting point
            if mask & (1 << i): # i in the subset
                for j in range(n):
                    if mask & (1 << j): # j in the subset
                        dp[i][mask] = min(dp[i][mask], dp[j][mask ^ (1 << i)] + dist[j][i])

    mask = (1 << n) - 1 # [1, 1, 1, ..., 1]
    last = np.argmin(dp[:, mask]) # the last point
    path = [last]

    # backtracking
    while mask != 1:
        for i in range(n):
            if mask & (1 << i) and dp[last][mask] == dp[i][mask ^ (1 << last)] + dist[i][last]:
                path.append(i)
                mask ^= (1 << last)
                last = i
                break

    path.reverse()
    return path

if __name__ == '__main__':
    targets = [[0, 0]
                ,[1.4619517395704253, 0.09071954914833391]
                ,[-4.355344841341103, -1.3582573064249779]
                ,[-1.470761899987032, 3.2386983630939987]
                ,[4.910248177123098, -3.6332638630323544]
                ,[-3.51003568699353, -4.95529217195066]
                ,[4.69722206076861, 4.565822721195147]
                ,[1.975005976204944, 0.486480825175196]
                ,[2.3459785684395227, 4.9590571590017145]
                ,[-3.303636718871059, -1.7042803895198677]
                ,[2.120409657428821, -2.8281907329784506]
                ]

    path = find_route(targets)
    print(path)