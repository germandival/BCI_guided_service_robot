# Jonas Braun
# jonas.braun@tum.de
# 12.01.2019

# process P300 data online

import numpy as np

from processing.process_offline import Dataset, Params


def filter_markers(markers, times, stims):
    """
    only return certain markers and the corresponding time stamps
    :param markers: whole marker stream
    :param times: correspoinding time stamps
    :param stims: markers to be searched for
    :return:
    """
    markers_filt = []
    times_filt = []
    for i, m in enumerate(markers):
        if m in stims:
            markers_filt.append(m)
            times_filt.append(times[i])
    return np.array(markers_filt), np.array(times_filt)


def get_labels(markers, times,  Params=Params()):
    """

    :param markers: complete marker stream of experiment
    :param times: correponding labels
    :param Params: instance of class Params() containing all relevant processing parameters
    :return: label: labels of all trials in experiment
    """
    target = Params.stim.target
    markers_rc, times_rc = filter_markers(markers, times, Params.stim.rows+Params.stim.columns)
    markers_target, times_target = filter_markers(markers, times, [target])

    markers_close = []
    for i, m in enumerate(markers_target):
        # get the marker value of the row collumn marker that is the closest in time to the target marker
        m_close = markers_rc[np.argmin(np.abs(times_rc-times_target[i]))]
        markers_close.append(m_close)

    label = np.zeros(len(markers_close) // 2 // Params.rep_per_trial)
    for i in range(len(markers_close) // 2 // Params.rep_per_trial):
        # returns 2 values: one is row stimulation (the lower one), other is column stimulation
        m_u = np.unique(markers_close[i*2*Params.rep_per_trial:(i+1)*2*Params.rep_per_trial])
        # take lower one and compare to all row stimulations to calculate row
        row = np.argmin(np.abs(Params.stim.rows-np.min(m_u)))
        # take higher one and compare to all column stimulations to calculate column
        column = np.argmin(np.abs(Params.stim.columns - np.max(m_u)))
        # calculate stimulation index
        label[i] = row * 3 + (column + 1)

    return label


def get_trial(eeg, time, markers, marker_time, Params=Params(), index=0):
    """
    cut out one trial out of whole experimental data
    :param eeg: eeg data of format n_samples x n_channels
    :param time: corresponding time stamps
    :param markers: whole marker stream
    :param marker_time: marker time stamps
    :param Params: instance of class Params() containing all relevant processing parameters
    :param index: number of trial to return
    :return:
    """
    trial_start = np.argwhere(markers == Params.stim.trial_start)
    trial_end = np.argwhere(markers == Params.stim.trial_end)
    trial_start = trial_start[index][0] - 5  # include 5 more markers
    trial_end = trial_end[index][0] + 5  # include 5 more markers
    markers = markers[trial_start:trial_end]
    marker_time = marker_time[trial_start:trial_end]
    trial_start = np.argmin(abs(time-marker_time[0])) - Params.Fs  # include 1s more data more data
    trial_end = np.argmin(abs(time-marker_time[-1])) + Params.Fs  # include 1s more data more data
    eeg = eeg[trial_start:trial_end, :]
    time = time[trial_start:trial_end]

    return eeg, time, markers, marker_time


class Model(Dataset):
    def __init__(self, Dataset):
        """
        class used for online classification. inherits from Dataset() to have consistent processing
        :param Dataset: preloaded and processed class Dataset() with offline classifier
        """
        super().__init__(filename=[], Params=Dataset.Params)

        self.DS_Offline = Dataset
        self.DS_test = None

        self.epochs_columns = []
        self.epochs_rows = []
        self.features_columns = []
        self.features_rows = []
        self.result = []

        self.clf_result_columns = np.array([1.0, 1.0, 1.0])
        self.clf_result_rows = np.array([1.0, 1.0, 1.0])

    def load(self):
        pass

    def assemble_features_online(self):
        """
        calculate features matrix for online classification of one trial
        :return:
        """
        for stim in self.Params.stim.columns:
            epochs, features = self.calculate_features(stimulation=stim)
            self.epochs_columns.append(epochs)
            self.features_columns.append(features)

        for stim in self.Params.stim.rows:
            epochs, features = self.calculate_features(stimulation=stim)
            self.epochs_rows.append(epochs)
            self.features_rows.append(features)

    def classify_all(self):
        """
        classify all rows and columns and perform basyesian updating on row and column probabilities
        :return:
        """
        for i_clf, features in enumerate(self.features_columns):
            result = self.DS_Offline.predict_proba(features=features)
            # take the mean of all classifications if there were multiple
            this = np.mean(result)
            # probability of all other columns
            others = (1 - this) / 2
            # accumulate evidence for every column
            for i_save in range(len(self.clf_result_columns)):
                if i_clf == i_save:
                    self.clf_result_columns[i_save] = self.clf_result_columns[i_save] * this
                else:
                    self.clf_result_columns[i_save] = self.clf_result_columns[i_save] * others
            # normalise to be valid probability mass function
            self.clf_result_columns = self.clf_result_columns / np.sum(self.clf_result_columns)

        for i_clf, features in enumerate(self.features_rows):
            result = self.DS_Offline.predict_proba(features=features)
            # take the mean of all classifications if there were multiple
            this = np.mean(result)  # avg = np.mean(result)
            # probability of all other columns
            others = (1 - this) / 2
            # accumulate evidence for every column
            for i_save in range(len(self.clf_result_rows)):
                if i_clf == i_save:
                    self.clf_result_rows[i_save] = self.clf_result_rows[i_save] * this
                else:
                    self.clf_result_rows[i_save] = self.clf_result_rows[i_save] * others
            # normalise to be valid probability mass function
            self.clf_result_rows = self.clf_result_rows / np.sum(self.clf_result_rows)

    def interpret_classification(self):
        """
        calculate the most likele index of box from most likely row and column
        :return:
        """
        # get prediction of item
        row = np.argmax(self.clf_result_rows)
        column = np.argmax(self.clf_result_columns)
        # calculate the index of the field on the display
        index = row * 3 + (column + 1)
        # return the minimum out of row and column probability as certainty
        certainty = np.min([self.clf_result_rows[row], self.clf_result_columns[column]])
        return index, certainty

    def classify_online(self, data, time, markers, marker_time, n_rep=0):
        """
        classify one online trial
        :param data: eeg data of format n_samples x n_channels
        :param time: corresponding time stamps
        :param markers: whole marker stream
        :param marker_time: marker time stamps
        :param n_rep: the how manieth repetition of that trial it is. if >0, evidence will be added to current
                    evidence. if 0, previous evidence will be deleted
        :return:
        """
        self.eeg = data
        self.time_stamps = np.array(time)
        self.markers = np.array(markers)
        self.time_markers = np.array(marker_time)

        self.epochs_columns = []
        self.epochs_rows = []
        self.features_columns = []
        self.features_rows = []

        if not n_rep:
            # if a new selection is to be made and not an old one to be update, delete previous evidence
            self.clf_result_columns = np.array([1.0, 1.0, 1.0])
            self.clf_result_rows = np.array([1.0, 1.0, 1.0])

        self.preprocess_data()
        self.assemble_features_online()
        self.classify_all()
        return self.interpret_classification()

    def test_classify_online(self, DS_test):
        """
        function used to test classifier in a pseudo online way. supply a preloaded dataset. this function will split
        it up into trials and will try to predict every trial
        :param DS_test: preloaded instance of class Dataset() whose trials are to be classified
        :return:
        """
        self.DS_test = DS_test
        self.labels = get_labels(self.DS_test.markers, self.DS_test.time_markers, self.DS_test.Params)
        self.result = []
        for i, label in enumerate(self.labels):
            eeg, time, markers, marker_time = get_trial(self.DS_test.eeg, self.DS_test.time_stamps, self.DS_test.markers,
                                                    self.DS_test.time_markers, self.DS_test.Params, index=i)
            result, cert = self.classify_online(eeg, time, markers, marker_time)
            self.result.append(result)

        return np.sum(np.array(self.labels) == np.array(self.result))

