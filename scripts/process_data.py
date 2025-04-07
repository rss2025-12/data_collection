import numpy as np
import csv
import matplotlib.pyplot as plt
import os

class DataReader():
    """
    reads data from the given csv file and plots desired quantities

    args:
        data_path: /path/to/csv
            path to csv file with data
        desired_plots: [[x0, [y00,y01,...], title0, filename0], [x1, [y10,y11,...], title1, filename1], ...]
            desired quantities to plot as a list of desired x axes, y quantities to plot, titles and filenames
    """
    def __init__(self, data_path, desired_plots):
        self.data_path = data_path
        self.desired_plots = desired_plots
        with open(self.data_path, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            headers = reader.fieldnames  # ['timestamp', 'position_error', 'yaw_error', 'cross_track_error']
            print(f"data in {self.data_path}:", headers)

            self.data = {header: [] for header in headers}

            for row in reader:
                for header in headers:
                    self.data[header].append(float(row[header]))
            
            for header in headers:
                self.data[header] = np.array(self.data[header])

    def plot(self):
        """
        create and display all desired plots.
        args: 
            None
        returns:
            None
        """
        cnt = 0
        for (x_csv, y_list_csv, title, filename) in self.desired_plots:
            print(f'creating plot {cnt}')
            x = self.data[x_csv]
            y_list = [self.data[y] for y in y_list_csv]
            varnames = y_list_csv
            plt.figure()
            for (y, varname) in zip(y_list, varnames):
                plt.plot(x, y, label=varname)
            plt.title(title)
            plt.legend()
            plt.savefig(filename)
            cnt += 1
        print('plots complete')
        plt.show()

if __name__== '__main__':
    data_path = os.path.join(os.path.dirname(__file__), '../data/lab_5/test.csv')
    filename1 = os.path.join(os.path.dirname(__file__), '../data/lab_5/cross_track_plot.png')
    filename2 = os.path.join(os.path.dirname(__file__), '../data/lab_5/position_error.png')
    filename2 = os.path.join(os.path.dirname(__file__), '../data/lab_5/yaw_error.png')
    desired_plots = [
        ['timestamp', ['cross_track_error'], 'Cross Track Error', filename1],
        ['timestamp', ['position_error'], 'Position Error', filename2]
        ['timestamp', ['yaw_error'], 'Yaw Error', filename3]
    ]
    reader = DataReader(data_path, desired_plots)
    print('created datareader object')
    reader.plot()
