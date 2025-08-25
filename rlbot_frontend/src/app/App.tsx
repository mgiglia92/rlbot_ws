import {Routes, Route} from "react-router-dom";
import ConfigSetup from "./ConfigSetup.tsx";
import Plotter from "./Plotter.tsx"

export default function App() {
  return (
    <Routes>
      <Route path = "/" element={<ConfigSetup />} />
      <Route path = "/plotter" element={<Plotter />} />
    </Routes>
  )
}