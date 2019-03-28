# Jonas Braun
# jonas.braun@tum.de
# 12.01.2019

# process P300 data offline


from scipy import io as sio
from scipy import signal as ss
import numpy as np
import matplotlib.pylab as plt
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis as LDA
from sklearn.svm import SVC as SVM
from sklearn.model_selection import cross_val_score as cvs

from recording.record_online import StimValues


def com_avf_reref(data):
    """

    :param data: eeg data of format n_samples x n_channels
    :return: data: eeg data of format n_samples x n_channels  with channel average subtracted
    """
    avg = np.tile(np.expand_dims(np.mean(data, -1), 1), (1, data.shape[-1]))
    return data - avg


def avg_repetitions(epochs, Params):
    """
    ensemble average across repetitinos of epochs in order to improve the signal to noise ratio
    :param epochs: epoched data of format n_samples, n_channels, n_repetitions
    :param Params: instance of class Params defining analysis params
    :return: epochs: epoched data of format n_samples, n_channels, n_repetitions / Params.n_rep
    """
    n_final = int(epochs.shape[-1]/Params.n_rep)
    epochs_avg = np.zeros((epochs.shape[0], epochs.shape[1], n_final))
    for i in range(n_final):
        epochs_avg[:, :, i] = np.mean(epochs[:, :, i*Params.n_rep:(i+1)*Params.n_rep], axis=-1)
    return epochs_avg


def bp_filter(data, Params):
    """

    :param data: eeg data of format n_samples x n_channels
    :param Params: instance of class Params defining analysis params
    :return: data_filt: eeg data of format n_samples x n_channels filtered with bandpass filter
    """
    order = Params.bp_order
    nyq = 0.5 * Params.Fs
    low = Params.f_low / nyq
    high = Params.f_high / nyq
    b, a = ss.butter(order, [low, high], btype='band')
    data_filt = ss.filtfilt(b, a, data, axis=0)
    return data_filt


def select_channels(data, Params):
    """

    :param data:  eeg data of format n_samples x n_channels
    :param Params: instance of class Params defining analysis params
    :return: data:  eeg data of format n_samples x n_selected_channels
    """
    return data[:, Params.channels]


def epoch_data(data, time_stamps, markers, marker_times, trigger, Params):
    """

    :param data:  eeg data of format n_samples x n_channels
    :param time_stamps: time stamp for every data point. must be same lenght as data
    :param markers: marker stream having e.g. target or non-target markers
    :param marker_times: time stamps of markers
    :param trigger: which marker to epoch to
    :param Params: instance of class Params defining analysis params
    :return: epochs: epoched data of format n_samples, n_channels, n_repetitions
    """
    epochs = np.empty((Params.n_in_epoch, data.shape[1], 0))
    times_trigger = marker_times[markers == trigger]

    for i_target, time in enumerate(times_trigger):
        index = np.argmin(np.abs(time_stamps-time))  # np.where(time_stamps == time)[0][0]
        epoch = np.expand_dims(data[index:index + Params.n_in_epoch, :], axis=2)
        epochs = np.append(epochs, epoch, axis=2)
        # epochs += [data[index:index + Params.n_in_epoch, :]]

    return epochs


def extract_features(epochs, Params):
    """
    assemple fearure matrix from epoched data
    :param epochs: epoched data of format n_samples, n_channels, n_repetitions
    :param Params: instance of class Params defining analysis params
    :return: features: np.array of shape n_repetitions, n_features
    """
    dec = ss.decimate(epochs, Params.decimate, axis=0)
    # n_final = int(np.floor(epochs.shape[0] / Params.decimate))
    # dec = np.zeros((n_final, epochs.shape[1], epochs.shape[2]))
    # for i in range(n_final):
    #     dec[i, :, :] = np.mean(epochs[i * Params.n_rep:(i + 1) * Params.n_rep, :, :], axis=0)

    return np.reshape(dec, (-1, dec.shape[2])).T  # will be of format n_epochs x n_features


class Params:
    def __init__(self):
        """
        class that defines all parameters of offline analysis
        """
        self.Fs = 128
        self.f_low = 1.0
        self.f_high = 20
        self.bp_order = 4

        self.channels = np.array([1, 2, 3, 12, 13, 14]) - 1  # 1, 3, 7, 8, 12, 14 = AF3, F3, O1, O2, F4, AF4 (original)
        # alternative with F7/8: 1, 2, 3, 12, 13, 14 (for reversed headset
        # channel names as given by Emotive
        # self.channel_names = np.array(['AF3', 'F7', 'F3', 'FC5', 'T7', 'P7', 'O1', 'O2', 'P8', 'T8',
        #                               'FC6', 'F4', 'F8', 'AF4'])
        # partially real channel names
        # self.channel_names = np.array(['P2', 'PO4', 'PO8', 'FC5', 'T7', 'P7', 'O1', 'O2', 'P8', 'T8',
        #                                'FC6', 'PO7', 'PO3', 'P1'])
        self.stim = StimValues()

        self.epoch = [0.0, 0.6]  # beginning and end of epoch relative to stimulation
        self.n_rep = 6  # how many epochs to use for ensemble averaging
        self.decimate = 8  # decimation factors

        self.rep_per_trial = 12  # how many repetitions have been recorded per trial
        self.n_trials = 10  # how many trials have been recorded oer experiment

    @property
    def n_in_epoch(self):
        return int(np.round((self.epoch[1] - self.epoch[0]) * self.Fs))

    @property
    def selected_channels_names(self):
        return self.channel_names[self.channels]


class Dataset:
    def __init__(self, filename=[], Params=Params()):
        """
        :param Params: instance of class Params
        :param filename: list containing filenames of datasets to be loaded
        """
        self.Params = Params
        self.filename = filename

        self.eeg = None
        self.time_stamps = None
        self.markers = None
        self.time_markers = None
        self.age = None
        self.gender = None

        self.times_target = None
        self.times_notarget = None
        self.epoch_target = None
        self.epoch_notarget = None
        self.features_target = None
        self.features_notarget = None
        self.features = None
        self.labels = None
        self.clf = None
        self.cv_scores = None

    @property
    def n_channels(self):
        return self.eeg.shape[1]

    def load_data(self):
        """
        load data from file into class
        :return: self
        """
        self.eeg = []
        self.time_stamps = []
        self.markers = []
        self.time_markers = []
        self.age = []
        self.gender = []

        if not len(self.filename):
            raise NameError('Error: no file to load was specified.')

        for i, file in enumerate(self.filename):
            mydict = sio.loadmat(file)
            self.eeg.append(mydict['X'])
            self.time_stamps.append(mydict['time_stamps'].T)
            self.markers.append(mydict['markers'])
            self.time_markers.append(mydict['time_markers'].T)
            self.age.append(mydict['age'][0])
            self.gender.append(mydict['gender'][0])
        # self.Params.Fs = mydict['Fs]  # currently not used because generation buggy

        if i == 0:
            self.eeg = self.eeg[0]
            self.markers = self.markers[0]
            self.time_stamps = self.time_stamps[0]
            self.time_markers = self.time_markers[0]
        elif i == 1:
            self.eeg = np.concatenate((self.eeg[0], self.eeg[1]), axis=0)
            self.markers = np.concatenate((self.markers[0], self.markers[1]), axis=0)
            offset = self.time_stamps[0][-1]
            self.time_stamps = np.concatenate((self.time_stamps[0],
                                               self.time_stamps[1] + offset), axis=0)
            self.time_markers = np.concatenate((self.time_markers[0],
                                               self.time_markers[1] + offset), axis=0)

    def preprocess_data(self):
        """
        apply predefined preprocessing steps
        :return:
        """
        # self.eeg = com_avf_reref(self.eeg)
        # apply bandpass filter
        self.eeg = bp_filter(self.eeg, self.Params)
        # select channels
        self.eeg = select_channels(self.eeg, self.Params)

    def calculate_features(self, stimulation):
        """
        assemble epochs and feature matrix for a certain stimulation. could be target or a certain row/column
        can be used both offline and online
        :param stimulation: the stimulation the features shall be based on
        :return: epochs, features
        """
        epochs = epoch_data(self.eeg, self.time_stamps, self.markers, self.time_markers,
                            stimulation, self.Params)
        epochs = avg_repetitions(epochs, self.Params)
        features = extract_features(epochs, self.Params)
        return epochs, features

    def assemble_features_offline(self):
        """
        calculate features and labels for offline classifier training
        :return:
        """

        self.epoch_target, self.features_target = self.calculate_features(self.Params.stim.target)
        self.epoch_notarget, self.features_notarget = self.calculate_features(self.Params.stim.notarget)

        self.features = np.concatenate((self.features_target, self.features_notarget), axis=0)
        self.labels = np.concatenate((np.ones((self.features_target.shape[0])),
                                     -1 * np.ones((self.features_notarget.shape[0]))), axis=0)

    def build_classifier(self):
        """
        build both and LDA and an SVM classifier for offline training. can lateron be used for online training
        :return:
        """
        self.clf = [LDA(n_components=None, priors=None, shrinkage='auto',
                        solver='eigen', store_covariance=False, tol=0.0001),
                    SVM(kernel='rbf', shrinking=True, probability=True, gamma='scale')]
        # self.clf.fit(self.features, self.labels)
        [c.fit(self.features, self.labels) for c in self.clf]
        # possible to use methods predict(X), predict_log_proba(X) or predict_proba()

        self.cv_scores = []
        [self.cv_scores.append(cvs(estimator=c, X=self.features, y=self.labels, cv=10, n_jobs=-1)) for c in self.clf]
        [print('mean cv score of clf {:d} is'.format(i), np.mean(cv)) for i, cv in enumerate(self.cv_scores)]

    def process(self):
        """
        summarise all sreps for offline processing
        :return:
        """
        self.load_data()
        self.preprocess_data()
        self.assemble_features_offline()
        self.build_classifier()

    def plot_target_avg(self):
        """
        plot averages of all channels used. one plot for target, one plot for non-target
        :return:
        """
        target = np.mean(self.epoch_target, axis=2)
        notarget = np.mean(self.epoch_notarget, axis=2)
        t = np.array(range(target.shape[0])) / self.Params.Fs + self.Params.epoch[0]

        fig, axes = plt.subplots(1, 2)
        axes[0].plot(t, target)
        axes[0].set_title('target')
        axes[1].plot(t, notarget)
        axes[1].set_title('no target')
        [ax.legend(self.Params.selected_channels_names) for ax in axes]
        [ax.set_xlabel('time after flashing (s)') for ax in axes]

    def plot_channel(self):
        """
        plot mean +- std for ever channel in a seperate subfig. different colour for target and non-target
        :return:
        """
        n_channels = len(self.Params.channels)
        t = np.array(range(self.epoch_notarget.shape[0])) / self.Params.Fs + self.Params.epoch[0]
        fig, axes = plt.subplots(1, n_channels)

        for i_ch in range(n_channels):
            signal = np.squeeze(self.epoch_target[:, i_ch, :])
            mean = np.mean(signal, axis=-1)
            std = np.std(signal, axis=-1)
            axes[i_ch].plot(t, mean, 'g-')
            axes[i_ch].fill_between(t, mean-std, mean+std, facecolor='green', alpha=0.5)
            signal = np.squeeze(self.epoch_notarget[:, i_ch, :])
            mean = np.mean(signal, axis=-1)
            std = np.std(signal, axis=-1)
            axes[i_ch].plot(t, mean, 'r-')
            axes[i_ch].fill_between(t, mean - std, mean + std, facecolor='red', alpha=0.5)
            axes[i_ch].set_xlabel('time after flashing (s)')
            axes[i_ch].set_title('channel '+self.Params.selected_channels_names[i_ch])

    def predict(self, features, labels=None):
        """
        predict one or multiple trials based on the offline classifier
        :param features: features to be classified n_trials x n_features
        :param labels: labels to test the results agains. not required
        :return: result: resulting labels of LDA classifier
        """
        result = [c.predict(features) for c in self.clf]

        if labels is not None:
            [print('accuracy is {}'.format(np.sum(labels == r) / len(labels))) for r in result]

        return result[0]

    def predict_proba(self, features, labels=None):
        """
        predict probabilities instead of class labels
        :param features: features to be classified n_trials x n_features
        :param labels: labels to test the results agains. not required
        :return: prob_target: probability of targtet as oredicted by LDA classifier
        """
        classes = [c.predict_proba(features) for c in self.clf]
        classes = classes[0]
        prob_target = classes[:, 1]
        return prob_target




