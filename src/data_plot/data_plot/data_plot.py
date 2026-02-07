import rclpy
from rclpy.node import Node
from base_msgs.msg import PlotInfo
import numpy as np
import matplotlib.pyplot as plt


class PlotData(Node):
    def __init__(self):
        super().__init__("data_plot_node")
        self.get_logger().info("data_plot_node created")

        # 订阅绘图信息
        self.subscription = self.create_subscription(
            PlotInfo,
            "planning/plot_info",
            self.do_plot,
            10,
        )

    # 绘图函数
    def do_plot(self, plot_info):
        plt.clf()  # 清空绘图区

        if not plot_info.trajectory_info.local_trajectory:  # 确保数组不为空
            self.get_logger().warn("Received empty trajectory info")
            return

        # 从已有数据创建数组
        s = np.asarray(
            [point.path_point.s for point in plot_info.trajectory_info.local_trajectory]
        )
        l = np.asarray(
            [point.path_point.l for point in plot_info.trajectory_info.local_trajectory]
        )
        dl_ds = np.asarray(
            [
                point.path_point.dl_ds
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )
        theta = np.asarray(
            [
                point.path_point.theta
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )
        kappa = np.asarray(
            [
                point.path_point.kappa
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )
        t = np.asarray(
            [
                point.speed_point.t
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )
        s_2path = np.asarray(
            [
                point.speed_point.s_2path
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )
        speed = np.asarray(
            [
                point.speed_point.speed
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )
        acceleration = np.asarray(
            [
                point.speed_point.acceleration
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )

        # 创建子图
        fig1 = plt.subplot(4, 1, 1)  # 4行1列的子图，第1个
        fig2 = plt.subplot(4, 1, 2)  # 4行1列的子图，第2个
        fig3 = plt.subplot(4, 1, 3)
        fig4 = plt.subplot(4, 1, 4)

        # -------------------------- 选择fig1 --------------------------
        plt.sca(fig1)
        plt.plot(s, l, color="green", label="l", linestyle="solid")  # 绘制l关于s的图

        # 绘制障碍物
        for obs in plot_info.obs_info:
            # 计算障碍物上下左右的边界
            s_left = obs.s - obs.obs_length / 2.0
            s_right = obs.s + obs.obs_length / 2.0
            l_bottom = obs.l - obs.obs_width / 2.0
            l_up = obs.l + obs.obs_width / 2.0

            # 绘制障碍物多边形
            p_sl = plt.Polygon(
                xy=[
                    [s_left, l_bottom],
                    [s_right, l_bottom],
                    [s_right, l_up],
                    [s_left, l_up],
                ],  # 传入四个顶点坐标
                color="blue",
                alpha=0.8,
            )

            fig1.add_patch(p_sl)  # 添加这个多边形到绘图区域

        plt.title("ls info")  # 图的标题
        plt.xlabel("s")  # 横坐标名称为s
        plt.legend()  # 图例

        # -------------------------- 选择fig2 --------------------------
        plt.sca(fig2)
        plt.plot(s, dl_ds, color="red", label="dl_ds", linestyle="solid")
        plt.plot(s, theta, color="orange", label="theta", linestyle="solid")
        plt.plot(s, kappa, color="cyan", label="kappa", linestyle="solid")
        plt.title("ls params")  # 图的标题
        plt.xlabel("s")  # 横坐标名称为s
        plt.legend()  # 图例

        # -------------------------- 选择fig3 --------------------------
        plt.sca(fig3)
        plt.plot(t, s_2path, color="green", label="s_2path", linestyle="solid")

        # 绘制障碍物
        for obs in plot_info.obs_info:
            # 计算障碍物的4个坐标点
            delta_s = obs.ds_dt_2path * (obs.t_out - obs.t_in)
            s_left_buttom = obs.s_2path - obs.obs_length / 2.0
            s_left_up = obs.s_2path + obs.obs_length / 2.0
            s_right_buttom = s_left_buttom + delta_s
            s_right_up = s_left_up + delta_s

            # 绘制障碍物多边形
            p_st = plt.Polygon(
                xy=[
                    [obs.t_in, s_left_buttom],
                    [obs.t_out, s_right_buttom],
                    [obs.t_out, s_right_up],
                    [obs.t_in, s_left_up],
                ],  # 传入四个顶点坐标
                color="blue",
                alpha=0.8,
            )

            fig3.add_patch(p_st)  # 添加这个多边形到绘图区域

        plt.title("st info")  # 图的标题
        plt.xlabel("t")  # 横坐标名称为s
        plt.legend()  # 图例

        # -------------------------- 选择fig4 --------------------------
        plt.sca(fig4)
        plt.plot(t, speed, color="red", label="speed", linestyle="solid")
        plt.plot(t, acceleration, color="cyan", label="acceleration", linestyle="solid")
        plt.title("ls params")  # 图的标题
        plt.xlabel("s")  # 横坐标名称为s
        plt.legend()  # 图例

        plt.pause(
            0.05
        )  # 暂停一小段时间，用于显示图像，不会阻塞  （触发matplotlib的绘图渲染流程，才能把绘制的图形显示在屏幕上，0.05s足够了）


def main(args=None):
    rclpy.init(args=args)
    plot_node = PlotData()

    try:
        rclpy.spin(plot_node)
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        rclpy.shutdown()  # 防止按ctrl+c时无法关闭


if __name__ == "__main__":
    main()
