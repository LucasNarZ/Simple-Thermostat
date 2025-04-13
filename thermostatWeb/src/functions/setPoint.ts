import axios from "axios"

export function changeSetPoint(newValue:number){
    const endpoint = import.meta.env.VITE_API_ENDPOINT
    axios.post(endpoint + "/shadow", {
        deviceName:"testEsp",
        shadow:{
            state:{
                desired:{
                  setPoint: newValue
                }
              }
        }
    })
}

export async function changeStatus(){
    const endpoint = import.meta.env.VITE_API_ENDPOINT
    const currentState = (JSON.parse(await getDeviceShadow())).state.reported.status;
    let newStatus;
    if(currentState == "Off"){
      newStatus = "On"
    }else{
      newStatus = "Off"
    }

    await axios.post(endpoint + "/shadow", {
        deviceName:"testEsp",
        shadow:{
            state:{
                desired:{
                  status: newStatus
                }
              }
        }
    })
}

export async function getDeviceShadow(){
  const endpoint = import.meta.env.VITE_API_ENDPOINT
  const response = await axios.get(endpoint + "/shadow");
  return response.data
}