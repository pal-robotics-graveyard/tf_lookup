
def key_from_transform(target, source):
    return "{}@{}".format(target.lstrip("/"), source.lstrip("/"))
