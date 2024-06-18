import { GoFileDirectory } from "react-icons/go";
import { CiPlay1 } from "react-icons/ci";
import "./Aside.scss";
import { useState } from "react";
import { Outlet, useNavigate } from "react-router-dom";

export const Aside = () => {
  const [open, setOpen] = useState<boolean>(false);
  const navigate = useNavigate();
  return (
    <div className="aside">
      <div className="aside-points">
        <GoFileDirectory
          size={24}
          onClick={() => {
            setOpen(!open);
            navigate("file");
          }}
        />
        <CiPlay1 size={24} onClick={() => setOpen(!open)} />
      </div>
      {open && (
        <div className="aside-additional">
          <Outlet />
        </div>
      )}
    </div>
  );
};
