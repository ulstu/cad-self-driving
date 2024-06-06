import { useCallback, useMemo } from "react";
import { useDropzone } from "react-dropzone";
import useAsideStore from "../../store/AsideStore";
import { shallow } from "zustand/shallow";

export const FileTab = () => {
  const decoder = useMemo(() => new TextDecoder("utf-8"), []);

  const { setCode } = useAsideStore((state) => state, shallow);

  const onDrop = useCallback(
    (acceptedFiles: File[]) => {
      acceptedFiles[0]
        .arrayBuffer()
        .then((item) => decoder.decode(new Uint8Array(item)))
        .then((item) => {
          setCode(item);
        });
    },
    [decoder, setCode]
  );

  const { getRootProps, getInputProps, isDragActive } = useDropzone({ onDrop });

  return (
    <div className="control-section">
      <div {...getRootProps()} className="dropbox">
        <input {...getInputProps()} />
        {isDragActive ? (
          <p>Drop the files here ...</p>
        ) : (
          <p>Переместите или выберете файл JSON</p>
        )}
      </div>
    </div>
  );
};
