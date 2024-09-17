class CustomCommandAction:
    def __init__(self, type: str, content: str):
        self.type = type
        self.content = content

    def __eq__(self, other):
        return self.type == other.type and self.content == other.content
