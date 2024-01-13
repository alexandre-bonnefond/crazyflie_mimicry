import multiprocessing
import time
import pose_input
import flock_control



def producer(queue):
    for i in range(10):
        queue.put(i)
    queue.put(None)


def consumer(pos):
    while True:
        # item = queue.get()
        item = pos[0]
        if item is None:
            break
        print(item)
        time.sleep(1)


if __name__ == '__main__':
    # A = multiprocessing.Array('d', [-1, -1])
    # q = multiprocessing.Queue()
    p1 = multiprocessing.Process(target=pose_input.detect_wrists_pos)
    # p2 = multiprocessing.Process(target=consumer, args=(pose_input.wrists_pos,))
    p2 = multiprocessing.Process(target=flock_control.test_run_sequence_multiprocess)
    # p2 = multiprocessing.Process(target=flock_control.run_seq_test)
    p1.start()
    p2.start()
    # p1.join()
    # p2.join()
