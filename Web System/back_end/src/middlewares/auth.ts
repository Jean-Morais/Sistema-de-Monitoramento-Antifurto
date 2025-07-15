// src/middlewares/authMiddleware.ts
import { Request, Response, NextFunction } from 'express';
import dotenv from 'dotenv'
import jwt from 'jsonwebtoken';

dotenv.config();

const secretKey = process.env.JWT_SECRET as string;

export interface AuthRequest extends Request {
  user?: any; // você pode tipar melhor se souber o conteúdo do token
}

export const authenticateToken = (req: AuthRequest, res: Response, next: NextFunction) => {
  const authHeader = req.headers['authorization'];
  const token = authHeader?.split(' ')[1]; // Bearer <token>

  if (!token) {
    return res.status(401).json({ message: 'Token não fornecido' });
  }

  jwt.verify(token, secretKey, (err, decoded) => {
    if (err) {
      return res.status(403).json({ message: 'Token inválido ou expirado' });
    }

    req.user = decoded;
    next();
  });
};
