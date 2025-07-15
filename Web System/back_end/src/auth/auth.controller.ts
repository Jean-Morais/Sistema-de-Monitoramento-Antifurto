import { Request, Response } from "express";
import { authService } from "./auth.service";

async function auth(req: Request, res: Response) {
    const {email,password} = req.body;
    try {
        const token = await authService.auth(email,password);
        if(token) {
            res.json({
                token : token
            })
        } else {
            res.status(401).json({
                message : 'Email ou senha invalidos',
            })   
        }
    } catch (error){
        res.status(500).json({
            message : 'Error no servidor',
            error
        }) 
    }
}

async function authRegister(req: Request, res: Response) {
    const {email,name,password} = req.body;
    try {
        const user = await authService.authRegister(email,name,password);
        res.status(201).json({
            message : 'Sucesso no registro',
            user
        })
    } catch (error) {
        res.json({
            message : 'Error no servidor',
            error
        })
    }
}

export const authController = {
    auth,
    authRegister
}