import mimetypes

header, encoded = "data:audio/wav;base64,sdffffffrwe".split(",", 1)
mime_type = header.split(";")[0][5:]
ext = mimetypes.guess_extension("audio/x-wav")
content_type = "image" if mime_type.startswith("image/") else \
                "audio" if mime_type.startswith("audio/") else None

print(mime_type,ext, content_type,mimetypes._db.types_map[0])