import { useState } from 'react';
import { SyntheticEvent } from 'react';
import Box from '@mui/material/Box';
import Slider from '@mui/material/Slider';
import Typography from '@mui/material/Typography';
import "./App.css"
import { changeSetPoint, changeStatus } from "./functions/setPoint"
const MAX = 30;
const MIN = 5;
const marks = [
  {
    value: MIN,
    label: '',
  },
  {
    value: MAX,
    label: '',
  },
];

function App() {
  const [val, setVal] = useState<number>(MIN)
  const handleChange = (_: Event | SyntheticEvent<Element, Event>, newValue: number) => {
    changeSetPoint(newValue);
    setVal(newValue);
  };
  const handlePower = () => {
    changeStatus()
  }

  return (
    <div className='main'>
    <Box sx={{ width: 250 }}>
      <Slider
        marks={marks}
        step={0.5}
        value={val}
        valueLabelDisplay="auto"
        min={MIN}
        max={MAX}
        onChangeCommitted={handleChange}
      />
      <Box sx={{ display: 'flex', justifyContent: 'space-between' }}>
        <Typography
          variant="body2"
          onClick={() => setVal(MIN)}
          sx={{ cursor: 'pointer' }}
        >
          {MIN} min
        </Typography>
        <Typography
          variant="body2"
          onClick={() => setVal(MAX)}
          sx={{ cursor: 'pointer' }}
        >
          {MAX} max
        </Typography>
      </Box>
    </Box>
    <img src='power.svg' className="power" onClick={handlePower}/>

    </div>
  )
}

export default App
