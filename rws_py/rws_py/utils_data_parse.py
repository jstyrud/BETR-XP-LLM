import numpy as np


def parse_rw_log_content(log_content):
    lines = log_content.split("\n")

    # first line is labels
    lables = lines[0].split("\r")[0].split("\t")
    lines = lines[1:]
    n_col = len(lines[0].split("\t"))
    data_mat = np.zeros((len(lines), n_col))
    remove_last_row = False
    for i_l in range(0, len(lines)):
        line = lines[i_l].split("\r")[0].split("\t")
        if len(line) > 1:
            for i_v, val in enumerate(line):
                data_mat[i_l, i_v] = float(val)
        elif i_l == len(lines) - 1:
            remove_last_row = True
    if remove_last_row:
        data_mat = np.delete(data_mat, data_mat.shape[0] - 1, 0)
    return data_mat, lables


def remove_zero_diff_start_and_end(
    data_mat, columns_to_use_for_diff, remove_start=True, remove_end=True, threshold=1e-6
):
    diff = np.diff(data_mat[:, columns_to_use_for_diff], axis=0)
    non_zero_idx = (diff > threshold).any(axis=1)
    non_zero_idx = np.where(non_zero_idx)[0]
    end = data_mat.shape[0]
    if remove_end:
        end = np.min([non_zero_idx[-1] + 2, end])
    start = 0
    if remove_start:
        start = np.max([non_zero_idx[0] - 1, 0])
    return data_mat[start:end, :]


def join_file_path(path: str, file_name: str):
    if path[0] not in ["/", "$"]:
        raise ValueError("Failed to delete file. Path must start with / or $")

    if path[-1] == "/":
        file_path = path + file_name
    else:
        file_path = path + "/" + file_name

    return file_path
