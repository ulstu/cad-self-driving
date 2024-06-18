import axios from "axios";

export class kppApi {
    get() {
        return axios.get('http://localhost:80')
    }
    post(data: JSON) {
        return axios.post('http://localhost:80', data, {
            headers: {
                'Content-Type': 'application/json',
            }
        })
    }
}