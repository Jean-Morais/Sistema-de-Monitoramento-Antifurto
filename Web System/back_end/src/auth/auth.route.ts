import { Router } from "express";
import { authController } from "./auth.controller";
import { validateBody } from "../middlewares/validate";
import { authSchema, authRegisterSchema } from "../schema/auth.schema";

const authRouter = Router()

authRouter.post('/auth',validateBody(authSchema),authController.auth);

authRouter.post('/auth/register',validateBody(authRegisterSchema),authController.authRegister);

export default authRouter;