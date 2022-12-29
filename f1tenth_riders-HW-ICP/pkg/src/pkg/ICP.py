import matplotlib.pyplot as plt
import numpy as np

class ICP:
    def __init__(self):
        #  ICP parameters
        self.EPS = 0.0001
        self.MAX_ITER = 20
        self.show_animation = False


    def icp_matching(self, previous_points, current_points):
        """
        Iterative Closest Point matching
        - input
        previous_points: 2D or 3D points in the previous frame
        current_points: 2D or 3D points in the current frame
        - output
        R: Rotation matrix
        T: Translation vector
        """
        H = None  # homogeneous transformation matrix

        dError = np.inf
        preError = np.inf
        count = 0

        # For plot
        if self.show_animation:
            fig = plt.figure()
            if previous_points.shape[0] == 3:
                fig.add_subplot(111, projection='3d')

        while dError >= self.EPS: # error값이 작아질 때까지 반복
            count += 1

            if self.show_animation:  # pragma: no cover
                self.plot_points(previous_points, current_points, fig)
                plt.pause(0.1)

            # 소스와 목적 점군 간에 가장 근처 이웃점 탐색. 계산량이 많음.
            indexes, error = self.nearest_neighbor_association(previous_points, current_points)

             # SVD 이용한 회전 행렬 계산
            Rt, Tt = self.svd_motion_estimation(previous_points[:, indexes], current_points)

            # 소스 점군에 변환행렬 적용해 좌표 갱신 --> 차원 축소 방법(SVD)를 이용해 변환행렬에 필요한 요소를 구한다.
            current_points = (Rt @ current_points) + Tt[:, np.newaxis]

            dError = preError - error

            if dError < 0:  # prevent matrix H changing, exit loop
                # print("Not Converge...", preError, dError, count)
                break

            preError = error
            H = self.update_homogeneous_matrix(H, Rt, Tt)

            if dError <= self.EPS: # update한 입실론이 Threshold(end condition)보다 작으면 ICP를 종료하고 아니면 다시 돌아간다. 
                # print("Converge", error, dError, count) 
                break
            elif self.MAX_ITER <= count:
                # print("Not Converge", error, dError, count)
                break

        R = np.array(H[0:-1, 0:-1])
        T = np.array(H[0:-1, -1])

        return R, T # R, T 반환


    def update_homogeneous_matrix(self, Hin, R, T):
        r_size = R.shape[0]
        H = np.zeros((r_size + 1, r_size + 1))

        H[0:r_size, 0:r_size] = R
        H[0:r_size, r_size] = T
        H[r_size, r_size] = 1.0

        if Hin is None:
            return H
        else:
            return Hin @ H

    def nearest_neighbor_association(self, previous_points, current_points):

        # calc the sum of residual errors
        delta_points = previous_points - current_points
        d = np.linalg.norm(delta_points, axis=0)
        error = sum(d)

        # calc index with nearest neighbor assosiation
        d = np.linalg.norm(np.repeat(current_points, previous_points.shape[1], axis=1)
                        - np.tile(previous_points, (1, current_points.shape[1])), axis=0)
                        
        indexes = np.argmin(d.reshape(current_points.shape[1], previous_points.shape[1]), axis=1)

        return indexes, error


    def svd_motion_estimation(self, previous_points, current_points):
        pm = np.mean(previous_points, axis=1)
        cm = np.mean(current_points, axis=1)

        p_shift = previous_points - pm[:, np.newaxis]
        c_shift = current_points - cm[:, np.newaxis]

        W = c_shift @ p_shift.T
        u, s, vh = np.linalg.svd(W)

        R = (u @ vh).T
        t = pm - (R @ cm)

        return R, t


    def plot_points(self, previous_points, current_points, figure):
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        
        plt.cla()
        plt.plot(previous_points[0, :], previous_points[1, :], ".r")
        plt.plot(current_points[0, :], current_points[1, :], ".b")
        plt.plot(0.0, 0.0, "xr")
        plt.axis("equal")


    def main(self, px_cur, py_cur, px_pre, py_pre, speed, steer):
        # motion = [speed*math.sin(steer), speed*math.cos(steer) ,steer*180/math.pi]  # movement [x[m],y[m],yaw[deg]]
        # print(f"ddddddd : {steer*180/math.pi}")

        # previous points
        px = px_pre
        py = py_pre

        previous_points = np.vstack((px, py))

        # current points ->> 움직인 양에 대해 current를 구함.(x,y) 벡터
        # cx = [math.cos(motion[2]) * x - math.sin(motion[2]) * y + motion[0]
        #     for (x, y) in zip(px, py)]
        # cy = [math.sin(motion[2]) * x + math.cos(motion[2]) * y + motion[0]
        #     for (x, y) in zip(px, py)]

        cx = px_cur
        cy = py_cur

        current_points = np.vstack((cx, cy))
        
        R, T = self.icp_matching(previous_points, current_points)
        
        return R, T