import multiprocessing
import pose_input
import flock_control


if __name__ == '__main__':
    p1 = multiprocessing.Process(target=pose_input.detect_wrists_pos)
    p2 = multiprocessing.Process(target=flock_control.test_run_sequence_multiprocess)
    p1.start()
    p2.start()
