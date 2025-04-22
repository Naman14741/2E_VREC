from typing import List


def customer_nearest_station(dist_matrix, cus_list: List[int], station_idx):
    """
    Find the nearest station to a customer from a list of customers.
    :param dist_matrix: The distance matrix.
    :param cus_list: List of customer indices.
    :param station_idx: Index of the station.
    :return: Index of the nearest customer to the station.
    """
    min_distance = float('inf')
    nearest_customer = -1

    for cus in cus_list:
        if dist_matrix[station_idx][cus] < min_distance:
            min_distance = dist_matrix[station_idx][cus]
            nearest_customer = cus

    return nearest_customer, min_distance

def station_nearest_customer(dist_matrix, station_list: List[int], cus_idx):
    """
    Find the nearest customer to a station from a list of stations.
    :param dist_matrix: The distance matrix.
    :param station_list: List of station indices.
    :param cus_idx: Index of the customer.
    :return: Index of the nearest station to the customer.
    """
    min_distance = float('inf')
    nearest_station = -1

    for station in station_list:
        if dist_matrix[station][cus_idx] < min_distance:
            min_distance = dist_matrix[station][cus_idx]
            nearest_station = station

    return nearest_station, min_distance