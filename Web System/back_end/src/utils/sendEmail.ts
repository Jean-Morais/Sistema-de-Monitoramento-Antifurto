import sgMail from '@sendgrid/mail';
import dotenv from 'dotenv'

dotenv.config()

sgMail.setApiKey(process.env.SENDGRID_API_KEY!);

export const sendAlertEmail = async (to: string, equipmentName: string) => {
  const msg = {
    to,
    from: process.env.SENDGRID_FROM as string, // precisa ser verificado no SendGrid
    subject: `Alerta: Equipamento "${equipmentName}" foi removido do alcance`,
    text: `O equipamento "${equipmentName}" saiu da sala.`,
    html: `<strong>O equipamento "${equipmentName}" foi retirado da sala sem autorização ou supervisão.</strong>`,
  };

  await sgMail.send(msg);
};
