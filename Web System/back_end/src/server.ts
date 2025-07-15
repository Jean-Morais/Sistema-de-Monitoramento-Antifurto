import app from "./app";
import dotenv from "dotenv"

dotenv.config();

//_deploy mode_
export default app;

//_dev mode_
/*app.listen(3000,()=>{
    console.log("Servidor rodando na porta 3000");
})
*/