# 1. Run Carla aerial view.
import aerialView as av
import TFManager as tf
import time
if __name__ == '__main__':

    try:
        av.main()
        time.sleep(3)
        tf.main()
        # main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
