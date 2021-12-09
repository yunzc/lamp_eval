from rosbag_io import read_lc_from_pg_bag
from file_io import read_traj_from_g2o
import key_handling as kh
import argparse

from experiment_utils import evaluate_g2o
from experiment_utils import evaluate_lc

datasets = ["tunnel", "urban", "finals", "ku", "subway"]

class LampFrontendResults:
    def __init__(self, traj_ate, lc_ate, lc_are, num_lc, lc_ir):
        self.traj_ate = traj_ate # trajectory ATE
        self.num_lc = num_lc # number of loop closures
        self.lc_ir = lc_ir # loop closure inlier ratio
    def __str__(self):
        return "{},{},{}".format(self.traj_ate, self.num_lc, self.lc_ir)

def main():
    parser = argparse.ArgumentParser(description="Evaluate lamp frontend results")
    parser.add_argument("path", type=str, help="path to folder with datasets and results.")
    parser.add_argument("label", type=str, help="label of experiment. (name)")
    args = parser.parse_args()

    for dataname in datasets:
        dataset_path = args.path + "/" + dataname
        exp_g2o = dataset_path + "/test_" + args.label + "/result.g2o"
        gt_g2o = dataset_path + "/ground_truth/result.g2o"
        exp_lc = dataset_path + "/test_" + args.label + "/loop_closures.bag"
        exp_lc_topic = "/base1/lamp/laser_loop_closures"

        robot_ate, robot_are = evaluate_g2o(exp_g2o, gt_g2o)

        lc_ate, lc_are = evaluate_lc(exp_lc, gt_g2o, exp_lc_topic)

        print(robot_ate)
        print(lc_ate)

if __name__ == "__main__":
    main()


