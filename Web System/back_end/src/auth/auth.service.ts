import prisma from "../prisma-client/prisma"
import bcrypt from 'bcrypt';
import jwt from 'jsonwebtoken';
import dotenv from "dotenv";

dotenv.config()

async function auth(email: string, password: string) {
    const user = await prisma.user.findUnique(
        {
            where : {
                email: email
            }
        }
    );
    if(!user) {
        return user;
    }
    const comparePassword = await bcrypt.compare(password,user.password);
    if(comparePassword) {
        const {password,...safeUser} = user;
        const token = await jwt.sign(safeUser,process.env.JWT_SECRET as string,{
            expiresIn: "1d"
        })
        return token;
    } else {
        return null;
    }
}

async function authRegister(email: string,name: string,unsafePassword: string) {
    const salt = await bcrypt.genSalt(10);
    const passwordHash = await bcrypt.hash(unsafePassword,salt);
    const user = await prisma.user.create({
        data: {
            name: name,
            email: email,
            password: passwordHash
        }
    });
    const {password,...safeUser} = user;
    return safeUser;
}

export const authService = {
    authRegister,
    auth
}