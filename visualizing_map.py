if __name__ == '__main__':
    from seacharts.enc import ENC

    # Values set in user-defined 'seacharts.yaml'
    # size = 9000, 5062
    # center = 44300, 6956450
    enc = ENC("seacharts.yaml")

    print(enc.seabed[10])
    print(enc.shore)
    print(enc.land)

    enc.display.show()